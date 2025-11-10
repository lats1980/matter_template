/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Ranging Requestor for Matter Template
 */

#include "channel_sounding_ras_initiator.h"

#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/cs_de.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

/* Known device MAC addresses from Kconfig */
static const uint8_t known_device1_mac[6] = {
    (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 40) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 32) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 24) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 16) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 8) & 0xFF),
    (uint8_t)(CONFIG_RFS_DEVICE1_MAC_ADDR & 0xFF)
};

static const uint8_t known_device2_mac[6] = {
    (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 40) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 32) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 24) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 16) & 0xFF),
    (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 8) & 0xFF),
    (uint8_t)(CONFIG_RFS_DEVICE2_MAC_ADDR & 0xFF)
};

#define CS_CONFIG_ID           0
#define NUM_MODE_0_STEPS       3
#define PROCEDURE_COUNTER_NONE (-1)
#define DE_SLIDING_WINDOW_SIZE (9)
#define MAX_AP                 (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

#define CHANNEL_SOUNDING_THREAD_STACK_SIZE 4096
#define CHANNEL_SOUNDING_THREAD_PRIORITY   5
#define CHANNEL_SOUNDING_SCAN_TIMEOUT_MS   2000

#define WAIT_AND_CHECK_CS_RESULT(cs_op_result)           \
    do {                                                 \
        (cs_op_result) = -EINPROGRESS;                   \
        k_sem_take(&sem_cs_control, K_FOREVER);          \
        if (cs_op_result) {                              \
            LOG_ERR("Channel sounding thread last operation failed: err %d", \
                    cs_op_result);                       \
            continue;                                    \
        }                                                \
    } while (0)

/* Thread and synchronization objects */
static K_THREAD_STACK_DEFINE(channel_sounding_thread_stack, CHANNEL_SOUNDING_THREAD_STACK_SIZE);
static struct k_thread channel_sounding_thread_data;
static k_tid_t channel_sounding_thread_id;

static K_SEM_DEFINE(sem_cs_control, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);
static K_MUTEX_DEFINE(cs_procedure_mutex);

static struct bt_conn *connection;
static struct bt_conn *auth_conn;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint32_t ras_feature_bits;

static uint8_t buffer_index;
static uint8_t buffer_num_valid;
static cs_de_dist_estimates_t distance_estimate_buffer[MAX_AP][DE_SLIDING_WINDOW_SIZE];

/* Channel Sounding control variables */
static int cs_op_result;
static bool cs_procedure_running = false;
static uint32_t cs_inactive_interval = CONFIG_RFS_SENSING_NORMAL_INACTIVE_INTERVAL_MS;
static bt_addr_le_t remote_addr_filter;

static enum channel_sounding_state cs_state = CS_STATE_UNINITIALIZED;

static channel_sounding_event_handler_t event_handler_cb;
struct k_work_delayable channel_sounding_work;

static bool set_channel_sounding_state(enum channel_sounding_state new_state);
static int channel_sounding_get_distance(float *distance);

static void store_distance_estimates(cs_de_report_t *p_report)
{
	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	for (uint8_t ap = 0; ap < p_report->n_ap; ap++) {
		memcpy(&distance_estimate_buffer[ap][buffer_index],
		       &p_report->distance_estimates[ap], sizeof(cs_de_dist_estimates_t));
	}

	buffer_index = (buffer_index + 1) % DE_SLIDING_WINDOW_SIZE;

	if (buffer_num_valid < DE_SLIDING_WINDOW_SIZE) {
		buffer_num_valid++;
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count/2] + values[count/2 - 1]) / 2;
	} else {
		return values[count/2];
	}
}

static cs_de_dist_estimates_t get_distance(uint8_t ap)
{
	cs_de_dist_estimates_t averaged_result = {};
	uint8_t num_ifft = 0;
	uint8_t num_phase_slope = 0;
	uint8_t num_rtt = 0;

	static float temp_ifft[DE_SLIDING_WINDOW_SIZE];
	static float temp_phase_slope[DE_SLIDING_WINDOW_SIZE];
	static float temp_rtt[DE_SLIDING_WINDOW_SIZE];

	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	for (uint8_t i = 0; i < buffer_num_valid; i++) {
		if (isfinite(distance_estimate_buffer[ap][i].ifft)) {
			temp_ifft[num_ifft] = distance_estimate_buffer[ap][i].ifft;
			num_ifft++;
		}
		if (isfinite(distance_estimate_buffer[ap][i].phase_slope)) {
			temp_phase_slope[num_phase_slope] =
				distance_estimate_buffer[ap][i].phase_slope;
			num_phase_slope++;
		}
		if (isfinite(distance_estimate_buffer[ap][i].rtt)) {
			temp_rtt[num_rtt] = distance_estimate_buffer[ap][i].rtt;
			num_rtt++;
		}
	}
	buffer_num_valid = 0;
	memset(distance_estimate_buffer, 0, sizeof(distance_estimate_buffer));
	buffer_index = 0;
	k_mutex_unlock(&distance_estimate_buffer_mutex);

	averaged_result.ifft = median_inplace(num_ifft, temp_ifft);
	averaged_result.phase_slope = median_inplace(num_phase_slope, temp_phase_slope);
	averaged_result.rtt = median_inplace(num_rtt, temp_rtt);

	return averaged_result;
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("Error when receiving ranging data with ranging counter %d (err %d)",
			ranging_counter, err);
		return;
	}

	if (ranging_counter != most_recent_local_ranging_counter) {
		LOG_INF("Ranging data dropped as peer ranging counter doesn't match local ranging "
			"data counter. (peer: %u, local: %u)",
			ranging_counter, most_recent_local_ranging_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
		return;
	}

	LOG_DBG("Ranging data received for ranging counter %d", ranging_counter);

	if (latest_local_steps.len == 0) {
		LOG_WRN("All subevents in ranging counter %u were aborted",
			most_recent_local_ranging_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);

		if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
			net_buf_simple_reset(&latest_peer_steps);
		}
		return;
	}

	/* This struct is static to avoid putting it on the stack (it's very large) */
	static cs_de_report_t cs_de_report;

	cs_de_populate_report(&latest_local_steps, &latest_peer_steps, BT_CONN_LE_CS_ROLE_INITIATOR,
			      &cs_de_report);

	net_buf_simple_reset(&latest_local_steps);

	if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
		net_buf_simple_reset(&latest_peer_steps);
	}

	k_sem_give(&sem_local_steps);

	cs_de_quality_t quality = cs_de_calc(&cs_de_report);

	if (quality == CS_DE_QUALITY_OK) {
		for (uint8_t ap = 0; ap < cs_de_report.n_ap; ap++) {
			if (cs_de_report.tone_quality[ap] == CS_DE_TONE_QUALITY_OK) {
				store_distance_estimates(&cs_de_report);
			}
		}
		k_sem_give(&sem_cs_control);
	}
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (most_recent_local_ranging_counter
		!= bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
		int sem_state = k_sem_take(&sem_local_steps, K_NO_WAIT);

		if (sem_state < 0) {
			dropped_ranging_counter = result->header.procedure_counter;
			LOG_INF("Dropped subevent results. Waiting for ranging data from peer.");
			return;
		}

		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* The steps from this subevent will not be used. */
	} else if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory to store step data. (%d > %d)",
				latest_local_steps.len + result->step_data_buf->len,
				latest_local_steps.size);
			net_buf_simple_reset(&latest_local_steps);
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
	}

	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		LOG_WRN("Procedure %u aborted", result->header.procedure_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
	}
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_DBG("Ranging data ready %i", ranging_counter);

	if (ranging_counter == most_recent_local_ranging_counter) {
		int err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps,
							  ranging_counter,
							  ranging_data_cb);
		if (err) {
			LOG_ERR("Get ranging data failed (err %d)", err);
			net_buf_simple_reset(&latest_local_steps);
			net_buf_simple_reset(&latest_peer_steps);
			k_sem_give(&sem_local_steps);
		}
	}
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data overwritten %i", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		cs_op_result = err;
	} else {
		LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
		cs_op_result = 0;
	}
	k_sem_give(&sem_cs_control);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	LOG_INF("The discovery procedure succeeded");

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	bt_gatt_dm_data_print(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
		cs_op_result = err;
		k_sem_give(&sem_cs_control);
		return;
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
		cs_op_result = err;
	}
	cs_op_result = 0;
	k_sem_give(&sem_cs_control);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_INF("The service could not be found during the discovery, disconnecting");
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_INF("The discovery procedure failed (err %d)", err);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
		cs_op_result = err;
	} else {
		LOG_INF("Security changed: %s level %u", addr, level);
		cs_op_result = 0;
	}
	k_sem_give(&sem_cs_control);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	/* Ignore peer parameter preferences. */
	return false;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_info info;

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (err %d)\n", err);
		return;
	}
	if (info.role == BT_CONN_ROLE_PERIPHERAL) {
		return;
	}

	/* Store the remote device address for device identification */
	const bt_addr_le_t *remote_addr = bt_conn_get_dst(conn);
	if (remote_addr) {
		LOG_INF("Confirmed connection to known RF sensing device");
	} else {
		LOG_ERR("Failed to get remote device address");
	}

	k_work_cancel_delayable(&channel_sounding_work);
	k_sem_give(&sem_cs_control);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != connection) {
		LOG_ERR("Disconnected from unknown connection");
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected (reason 0x%02X)", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	bt_conn_unref(conn);
	connection = NULL;
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
		cs_op_result = 0;
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
		cs_op_result = status;
	}
	k_sem_give(&sem_cs_control);
}

static void config_create_cb(struct bt_conn *conn,
			     uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS config creation complete. ID: %d", config->id);
		cs_op_result = 0;
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
		cs_op_result = status;
	}
	k_sem_give(&sem_cs_control);
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
		cs_op_result = 0;
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
		cs_op_result = status;
	}
	k_sem_give(&sem_cs_control);
}

static void procedure_enable_cb(struct bt_conn *conn,
				uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			LOG_INF("CS procedures enabled:\n"
				" - config ID: %u\n"
				" - antenna configuration index: %u\n"
				" - TX power: %d dbm\n"
				" - subevent length: %u us\n"
				" - subevents per event: %u\n"
				" - subevent interval: %u\n"
				" - event interval: %u\n"
				" - procedure interval: %u\n"
				" - procedure count: %u\n"
				" - maximum procedure length: %u",
				params->config_id, params->tone_antenna_config_selection,
				params->selected_tx_power, params->subevent_len,
				params->subevents_per_event, params->subevent_interval,
				params->event_interval, params->procedure_interval,
				params->procedure_count, params->max_procedure_len);
		} else {
			LOG_INF("CS procedures disabled.");
		}
		cs_op_result = 0;
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
		cs_op_result = status;
	}
	k_sem_give(&sem_cs_control);
}

void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
	if (err) {
		LOG_WRN("Error while reading RAS feature bits (err %d)", err);
	} else {
		LOG_INF("Read RAS feature bits: 0x%x", feature_bits);
		ras_feature_bits = feature_bits;
	}
	cs_op_result = err;
	k_sem_give(&sem_cs_control);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	if (!device_info) {
		LOG_ERR("No device info");
		return;
	}

	if (!device_info->recv_info) {
		LOG_ERR("No device recv info");
		return;
	}

	/* Filter out devices with weak signal strength */
	if (device_info->recv_info->rssi < CONFIG_RFS_FILTER_RSSI_THRESHOLD) {
		LOG_DBG("Device RSSI %d dBm below threshold %d dBm, ignoring",
			device_info->recv_info->rssi,
			CONFIG_RFS_FILTER_RSSI_THRESHOLD);
		return;
	}

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
	bt_scan_stop();

	struct bt_conn *conn = NULL;
	err = bt_conn_le_create(device_info->recv_info->addr,
				BT_CONN_LE_CREATE_CONN,
				device_info->conn_param, &conn);
	if (err) {
		LOG_ERR("Create conn failed (err %d)", err);
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		if (err) {
			LOG_ERR("Failed to restart scanning (err %i)", err);
		}
	} else {
		LOG_INF("Connection pending...");
		connection = bt_conn_ref(conn);
		bt_conn_unref(conn);
	}
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	int err;

	LOG_INF("Connecting failed, restarting scanning");

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Failed to restart scanning (err %i)", err);
		return;
	}
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	LOG_INF("Connecting");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	bt_addr_le_t addr;
	bt_addr_le_t zero_addr = {0};

	bt_scan_stop();
	bt_scan_filter_remove_all();

	if (memcmp(&remote_addr_filter, &zero_addr, sizeof(bt_addr_le_t)) != 0) {
		/* Add specified device address to the filter */
		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &remote_addr_filter);
		if (err) {
			LOG_ERR("Scanning filters cannot be set (err %d)", err);
			return err;
		}
		memset(&remote_addr_filter, 0, sizeof(remote_addr_filter));
	} else {
		LOG_INF("No remote address filter set, using known device addresses");
		/* Add known device addresses to the filter */
		addr.type = BT_ADDR_LE_RANDOM;
		memcpy(&addr.a.val, known_device1_mac, 6);
		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &addr);
		if (err) {
			LOG_ERR("Scanning filters cannot be set (err %d)", err);
			return err;
		}

		addr.type = BT_ADDR_LE_RANDOM;
		memcpy(&addr.a.val, known_device2_mac, 6);
		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &addr);
		if (err) {
			LOG_ERR("Scanning filters cannot be set (err %d)", err);
			return err;
		}
	}

	err = bt_scan_filter_enable(BT_SCAN_ADDR_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return 0;
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);

	if (!bonded) {
		LOG_WRN("Device not bonded, disconnecting: %s", addr);
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));

	LOG_WRN("Bonding required but pairing failed, disconnecting: %s", addr);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

static void channel_sounding_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int err;

	LOG_INF("Init Channel Sounding thread");

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization info callbacks.");
		return;
	}

	struct bt_scan_init_param param = {
		.scan_param = NULL, .conn_param = BT_LE_CONN_PARAM(0x10, 0x10, 0, BT_GAP_MS_TO_CONN_TIMEOUT(4000)), .connect_if_match = 0};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	while (1) {
		buffer_index = 0;
		buffer_num_valid = 0;
		memset(distance_estimate_buffer, 0, sizeof(distance_estimate_buffer));
		cs_op_result = 0;
		if (connection) {
			LOG_INF("Disconnecting existing connection");
			bt_ras_rreq_free(connection);
			bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}

		err = scan_init();
		if (err) {
			LOG_ERR("Scan init failed (err %d)", err);
		}
		set_channel_sounding_state(CS_STATE_STOPPED);
		if (cs_procedure_running) {
			k_work_schedule(&channel_sounding_work, K_MSEC(cs_inactive_interval));
		}
		k_sem_take(&sem_cs_control, K_FOREVER);
		set_channel_sounding_state(CS_STATE_CONNECTING);
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		if (err) {
			LOG_ERR("Scanning failed to start (err %i)", err);
			continue;
		}
		k_work_schedule(&channel_sounding_work, K_MSEC(CHANNEL_SOUNDING_SCAN_TIMEOUT_MS));
		// wait for connection
		k_sem_take(&sem_cs_control, K_FOREVER);
		if (!connection) {
			LOG_ERR("No connected device");
			continue;
		}
		err = bt_conn_set_security(connection, BT_SECURITY_L2);
		if (err) {
			LOG_ERR("Failed to encrypt connection (err %d)", err);
			continue;
		}
		cs_op_result = -EINPROGRESS;
		k_sem_take(&sem_cs_control, K_FOREVER);
		// wait and check security result
		if (cs_op_result) {
			LOG_ERR("Failed to secure connection: err %d", cs_op_result);
			continue;
		}

		static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};
		err = bt_gatt_exchange_mtu(connection, &mtu_exchange_params);
		if (err) {
			LOG_ERR("MTU exchange failed (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
		if (err) {
			LOG_ERR("Discovery failed (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = true,
			.enable_reflector_role = false,
			.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};
		err = bt_le_cs_set_default_settings(connection, &default_settings);
		if (err) {
			LOG_ERR("Failed to configure default CS settings (err %d)", err);
			continue;
		}
		err = bt_ras_rreq_read_features(connection, ras_features_read_cb);
		if (err) {
			LOG_ERR("Could not get RAS features from peer (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		const bool realtime_rd = ras_feature_bits & RAS_FEAT_REALTIME_RD;
		if (realtime_rd) {
			err = bt_ras_rreq_realtime_rd_subscribe(connection,
								&latest_peer_steps,
								ranging_data_cb);
			if (err) {
				LOG_ERR("RAS RREQ Real-time ranging data subscribe failed (err %d)", err);
				continue;
			}
		} else {
			err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
			if (err) {
				LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
				continue;
			}
			err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
			if (err) {
				LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
				continue;
			}
			err = bt_ras_rreq_on_demand_rd_subscribe(connection);
			if (err) {
				LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
				continue;
			}
			err = bt_ras_rreq_cp_subscribe(connection);
			if (err) {
				LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
				continue;
			}
		}

		err = bt_le_cs_read_remote_supported_capabilities(connection);
		if (err) {
			LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		struct bt_le_cs_create_config_params config_params = {
			.id = CS_CONFIG_ID,
			.main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
			.sub_mode_type = BT_CONN_LE_CS_SUB_MODE_1,
			.min_main_mode_steps = 2,
			.max_main_mode_steps = 5,
			.main_mode_repetition = 0,
			.mode_0_steps = NUM_MODE_0_STEPS,
			.role = BT_CONN_LE_CS_ROLE_INITIATOR,
			.rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
			.cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
			.channel_map_repetition = 1,
			.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
			.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
			.ch3c_jump = 2,
		};
		bt_le_cs_set_valid_chmap_bits(config_params.channel_map);
		err = bt_le_cs_create_config(connection, &config_params,
						 BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
		if (err) {
			LOG_ERR("Failed to create CS config (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		err = bt_le_cs_security_enable(connection);
		if (err) {
			LOG_ERR("Failed to start CS Security (err %d)", err);
			return;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);

		const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
			.config_id = CS_CONFIG_ID,
			.max_procedure_len = 1000,
			.min_procedure_interval = realtime_rd ? 5 : 10,
			.max_procedure_interval = realtime_rd ? 5 : 10,
			.max_procedure_count = 0,
			.min_subevent_len = 16000,
			.max_subevent_len = 16000,
			.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
			.phy = BT_LE_CS_PROCEDURE_PHY_2M,
			.tx_power_delta = 0x80,
			.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
			.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
			.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
		};
	
		err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
		if (err) {
			LOG_ERR("Failed to set procedure parameters (err %d)", err);
			continue;
		}

		struct bt_le_cs_procedure_enable_param params = {
			.config_id = CS_CONFIG_ID,
			.enable = 1,
		};
	
		err = bt_le_cs_procedure_enable(connection, &params);
		if (err) {
			LOG_ERR("Failed to enable CS procedures (err %d)", err);
			continue;
		}
		WAIT_AND_CHECK_CS_RESULT(cs_op_result);
		set_channel_sounding_state(CS_STATE_STARTED);
		LOG_INF("Channel Sounding started successfully");
		k_work_reschedule(&channel_sounding_work,
			      K_MSEC(CONFIG_RFS_SENSING_ACTIVE_INTERVAL_MS));
		// wait for stop request
		k_sem_take(&sem_cs_control, K_FOREVER);
		k_work_cancel_delayable(&channel_sounding_work);
		float distance;

		err = channel_sounding_get_distance(&distance);
		if (err == 0) {
			event_handler_cb(connection, distance);
		}
	}
}

void channel_sounding_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&sem_cs_control);
}

int channel_sounding_init(channel_sounding_event_handler_t event_handler)
{
	if (get_channel_sounding_state() != CS_STATE_UNINITIALIZED) {
		LOG_INF("Channel Sounding already initialized");
		return 0;
	}

	LOG_INF("Initializing Channel Sounding");

	event_handler_cb = event_handler;
	k_work_init_delayable(&channel_sounding_work, channel_sounding_work_handler);

	/* Create the Channel Sounding thread */
	channel_sounding_thread_id = k_thread_create(&channel_sounding_thread_data,
						      channel_sounding_thread_stack,
						      K_THREAD_STACK_SIZEOF(channel_sounding_thread_stack),
						      channel_sounding_thread_entry,
						      NULL, NULL, NULL,
						      CHANNEL_SOUNDING_THREAD_PRIORITY,
						      0, K_NO_WAIT);

	if (channel_sounding_thread_id == NULL) {
		LOG_ERR("Failed to create Channel Sounding thread");
		return -ENOMEM;
	}

	k_thread_name_set(channel_sounding_thread_id, "channel_sounding");
	LOG_INF("Channel Sounding thread created successfully");

	return 0;
}

enum channel_sounding_state get_channel_sounding_state(void)
{
	enum channel_sounding_state state;

	k_mutex_lock(&cs_procedure_mutex, K_FOREVER);
	state = cs_state;
	k_mutex_unlock(&cs_procedure_mutex);

	return state;
}

bool set_channel_sounding_state(enum channel_sounding_state state)
{
	bool success = false;

	k_mutex_lock(&cs_procedure_mutex, K_FOREVER);
	if (state == CS_STATE_STOPPED) {
		if (cs_state != CS_STATE_STOPPED) {
			cs_state = CS_STATE_STOPPED;
			success = true;
		}
	} else if (state == CS_STATE_CONNECTING) {
		if (cs_state == CS_STATE_STOPPED) {
			cs_state = CS_STATE_CONNECTING;
			success = true;
		}
	} else if (state == CS_STATE_STARTED)  {
		if (cs_state == CS_STATE_CONNECTING) {
			cs_state = CS_STATE_STARTED;
			success = true;
		}
	} else {
		LOG_ERR("Invalid Channel Sounding state requested");
	}
	if (!success) {
		LOG_DBG("Channel Sounding state change to %d not allowed from state %d",
			state, cs_state);
	} else {
		LOG_DBG("Channel Sounding state changed to %d", state);
	}
	k_mutex_unlock(&cs_procedure_mutex);

	return success;
}

int channel_sounding_procedure_enable(bool enable)
{
	if (get_channel_sounding_state() == CS_STATE_UNINITIALIZED) {
		LOG_ERR("Channel Sounding not initialized");
		return -ENOTCONN;
	}
	if (enable == cs_procedure_running) {
		LOG_INF("Channel Sounding procedures already %s",
			enable ? "enabled" : "disabled");
		return 0;
	}
	/* Notify the Channel Sounding thread to start/stop procedures */
	if (cs_procedure_running) {
		LOG_INF("Stopping Channel Sounding procedures");
		if (get_channel_sounding_state() == CS_STATE_STOPPED) {
			k_work_cancel_delayable(&channel_sounding_work);
		}
	} else {
		LOG_INF("Starting Channel Sounding procedures");
		k_sem_give(&sem_cs_control);
	}
	cs_procedure_running = enable;
	LOG_INF("Channel Sounding procedures %s", cs_procedure_running ? "enabled" : "disabled");

	return 0;
}

static int channel_sounding_get_distance(float *distance)
{
	if (!distance) {
		LOG_ERR("Invalid distance pointer");
		return -EINVAL;
	}

	if (get_channel_sounding_state() != CS_STATE_STARTED) {
		LOG_ERR("Channel Sounding not started");
		return -EINVAL;
	}

	if (buffer_num_valid == 0) {
		LOG_DBG("No valid distance estimates available");
		return -EINVAL;
	}

	/* Get distance estimates for antenna path 0 (first antenna) */
	cs_de_dist_estimates_t distance_on_ap = get_distance(0);
	LOG_DBG("Distance estimate: ifft: %f, "
		"phase_slope: %f, rtt: %f",
		(double)distance_on_ap.ifft,
		(double)distance_on_ap.phase_slope,
		(double)distance_on_ap.rtt);
	/* Check if IFFT estimate is valid (finite number) */
	if (isfinite(distance_on_ap.ifft)) {
		*distance = distance_on_ap.ifft;
	} else if (isfinite(distance_on_ap.phase_slope)) {
		*distance = distance_on_ap.phase_slope;
		LOG_WRN("IFFT distance estimate not available, using phase slope estimate instead");
	} else if (isfinite(distance_on_ap.rtt)) {
		*distance = distance_on_ap.rtt;
		LOG_WRN("IFFT and phase slope distance estimates not available, using RTT estimate instead");
	} else {
		LOG_DBG("No valid distance estimates available on antenna path 0");
		return -EINVAL;
	}

	LOG_DBG("Retrieved distance: %.2f", (double)*distance);

	return 0;
}

int channel_sounding_set_inactive_interval(uint32_t interval_ms)
{
	if (cs_inactive_interval == interval_ms) {
		LOG_INF("Channel Sounding inactive interval already set to %u ms", interval_ms);
		return 0;
	}
	cs_inactive_interval = interval_ms;
	LOG_INF("Channel Sounding inactive interval set to %u ms", cs_inactive_interval);
	if (get_channel_sounding_state() == CS_STATE_STOPPED && cs_procedure_running) {
		k_work_reschedule(&channel_sounding_work, K_MSEC(cs_inactive_interval));
	}
	return 0;
}

int channel_sounding_set_filter_by_addr(const bt_addr_le_t *addr)
{
	if (!addr) {
		LOG_ERR("Invalid address pointer");
		return -EINVAL;
	}
	memcpy(&remote_addr_filter, addr, sizeof(bt_addr_le_t));
	LOG_INF("Channel Sounding remote address filter set");

	return 0;
}
