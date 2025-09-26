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

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(channel_sounding, LOG_LEVEL_INF);

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

#define CON_STATUS_LED DK_LED1

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

/* Thread and synchronization objects */
static K_THREAD_STACK_DEFINE(channel_sounding_thread_stack, CHANNEL_SOUNDING_THREAD_STACK_SIZE);
static struct k_thread channel_sounding_thread_data;
static k_tid_t channel_sounding_thread_id;

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_ras_features, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);
static K_MUTEX_DEFINE(cs_procedure_mutex);

static struct bt_conn *connection;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint32_t ras_feature_bits;

/* Remote device address storage */
static bool remote_address_valid = false;
static uint8_t remote_device_addr[6];

static uint8_t buffer_index;
static uint8_t buffer_num_valid;
static cs_de_dist_estimates_t distance_estimate_buffer[MAX_AP][DE_SLIDING_WINDOW_SIZE];

/* Channel Sounding control variables */
static bool cs_initialized = false;
static bool cs_enabled = false;

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
		return;
	}

	LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	k_sem_give(&sem_mtu_exchange_done);
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
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
	}

	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_INF("The service could not be found during the discovery, disconnecting");
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_INF("The discovery procedure failed (err %d)", err);
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
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
		return;
	}

	LOG_INF("Security changed: %s level %u", addr, level);
	k_sem_give(&sem_security);
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
		remote_address_valid = false;
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (err %d)\n", err);
		remote_address_valid = false;
		return;
	}
	if (info.role == BT_CONN_ROLE_PERIPHERAL) {
		remote_address_valid = false;
		return;
	}

	connection = bt_conn_ref(conn);
	
	/* Store the remote device address for device identification */
	const bt_addr_le_t *remote_addr = bt_conn_get_dst(conn);
	if (remote_addr) {
		memcpy(remote_device_addr, remote_addr->a.val, 6);
		remote_address_valid = true;
		LOG_INF("Stored remote device address: %02X:%02X:%02X:%02X:%02X:%02X",
			remote_device_addr[5], remote_device_addr[4], remote_device_addr[3],
			remote_device_addr[2], remote_device_addr[1], remote_device_addr[0]);
		
		/* Double-check: if somehow an unknown device connected, disconnect it immediately */
		if (!channel_sounding_is_known_device(remote_device_addr)) {
			LOG_ERR("Connected to unknown device - disconnecting immediately");
			bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			return;
		}
		
		LOG_INF("Confirmed connection to known RF sensing device");
	} else {
		remote_address_valid = false;
		LOG_ERR("Failed to get remote device address");
	}

	k_sem_give(&sem_connected);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int err;
	
	LOG_INF("Disconnected (reason 0x%02X)", reason);

	if (conn != connection) {
		return;
	}

	bt_conn_unref(conn);
	connection = NULL;
	dk_set_led_off(CON_STATUS_LED);

	/* Reset CS enabled state and remote address on disconnect */
	k_mutex_lock(&cs_procedure_mutex, K_FOREVER);
	cs_enabled = false;
	k_mutex_unlock(&cs_procedure_mutex);
	
	remote_address_valid = false;
	memset(remote_device_addr, 0, sizeof(remote_device_addr));
	
	/* Brief delay to ensure cleanup is complete before restarting scan */
	k_sleep(K_MSEC(100));
	
	/* Restart scanning to look for known RF sensing devices */
	LOG_INF("Restarting scan to reconnect to RF sensing devices");
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Failed to restart scanning after disconnect (err %d)", err);
		/* Main thread will retry scanning in its loop */
	} else {
		LOG_INF("Scanning restarted successfully");
	}
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
		k_sem_give(&sem_remote_capabilities_obtained);
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn,
			     uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS config creation complete. ID: %d", config->id);
		k_sem_give(&sem_config_created);
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
		k_sem_give(&sem_cs_security_enabled);
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn,
				uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		k_mutex_lock(&cs_procedure_mutex, K_FOREVER);
		cs_enabled = (params->state == 1);
		k_mutex_unlock(&cs_procedure_mutex);

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
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	}
}

void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
	if (err) {
		LOG_WRN("Error while reading RAS feature bits (err %d)", err);
	} else {
		LOG_INF("Read RAS feature bits: 0x%x", feature_bits);
		ras_feature_bits = feature_bits;
	}

	k_sem_give(&sem_ras_features);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
	
	// Check if this is a known RF sensing device before allowing connection
	if (!channel_sounding_is_known_device(device_info->recv_info->addr->a.val)) {
		LOG_WRN("Ignoring connection to unknown device: %s", addr);
		// Continue scanning instead of connecting to unknown device
		return;
	}
	
	LOG_INF("Known RF sensing device detected: %s - allowing connection", addr);
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

	struct bt_scan_init_param param = {
		.scan_param = NULL, .conn_param = BT_LE_CONN_PARAM_DEFAULT, .connect_if_match = 1};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return 0;
}

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

	LOG_INF("Starting Channel Sounding thread");

	err = scan_init();
	if (err) {
		LOG_ERR("Scan init failed (err %d)", err);
		return;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %i)", err);
		return;
	}
restart:
	k_sem_take(&sem_connected, K_FOREVER);

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if (err) {
		LOG_ERR("Failed to encrypt connection (err %d)", err);
		bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		goto restart;
	}

	k_sem_take(&sem_security, K_FOREVER);

	static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

	bt_gatt_exchange_mtu(connection, &mtu_exchange_params);

	k_sem_take(&sem_mtu_exchange_done, K_FOREVER);

	err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("Discovery failed (err %d)", err);
		return;
	}

	k_sem_take(&sem_discovery_done, K_FOREVER);

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		LOG_ERR("Failed to configure default CS settings (err %d)", err);
		return;
	}

	err = bt_ras_rreq_read_features(connection, ras_features_read_cb);
	if (err) {
		LOG_ERR("Could not get RAS features from peer (err %d)", err);
		return;
	}

	k_sem_take(&sem_ras_features, K_FOREVER);

	const bool realtime_rd = ras_feature_bits & RAS_FEAT_REALTIME_RD;

	if (realtime_rd) {
		err = bt_ras_rreq_realtime_rd_subscribe(connection,
							&latest_peer_steps,
							ranging_data_cb);
		if (err) {
			LOG_ERR("RAS RREQ Real-time ranging data subscribe failed (err %d)", err);
			return;
		}
	} else {
		err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
		if (err) {
			LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
			return;
		}

		err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
		if (err) {
			LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
			return;
		}

		err = bt_ras_rreq_on_demand_rd_subscribe(connection);
		if (err) {
			LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
			return;
		}

		err = bt_ras_rreq_cp_subscribe(connection);
		if (err) {
			LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
			return;
		}
	}

	err = bt_le_cs_read_remote_supported_capabilities(connection);
	if (err) {
		LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
		return;
	}

	k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

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
		.channel_map_repetition = 3,
		.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
		.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
		.ch3c_jump = 2,
	};

	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	err = bt_le_cs_create_config(connection, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Failed to create CS config (err %d)", err);
		return;
	}

	k_sem_take(&sem_config_created, K_FOREVER);

	err = bt_le_cs_security_enable(connection);
	if (err) {
		LOG_ERR("Failed to start CS Security (err %d)", err);
		return;
	}

	k_sem_take(&sem_cs_security_enabled, K_FOREVER);

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = 1000,
		.min_procedure_interval = realtime_rd ? 5 : 10,
		.max_procedure_interval = realtime_rd ? 5 : 10,
		.max_procedure_count = 0,
		.min_subevent_len = 60000,
		.max_subevent_len = 60000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_1M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters (err %d)", err);
		return;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	err = bt_le_cs_procedure_enable(connection, &params);
	if (err) {
		LOG_ERR("Failed to enable CS procedures (err %d)", err);
		return;
	}
	LOG_INF("Channel Sounding initialization complete");

	/* Main processing loop */
	while (true) {
		k_sleep(K_MSEC(3000));
		/* If connection is lost, restart scanning */
		if (connection == NULL) {
			goto restart;
		}
		LOG_DBG("Channel Sounding thread processing...");
	}
}

int channel_sounding_init(void)
{
	if (cs_initialized) {
		LOG_INF("Channel Sounding already initialized");
		return 0;
	}

	LOG_INF("Initializing Channel Sounding");

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

	cs_initialized = true;
	LOG_INF("Channel Sounding thread created successfully");

	return 0;
}

int channel_sounding_procedure_enable(bool enable)
{
	int err;

	if (!cs_initialized) {
		LOG_ERR("Channel Sounding not initialized");
		return -ENOTCONN;
	}

	if (!connection) {
		LOG_ERR("No active Bluetooth connection");
		return -ENOTCONN;
	}

	k_mutex_lock(&cs_procedure_mutex, K_FOREVER);

	if (cs_enabled == enable) {
		LOG_INF("Channel Sounding already %s", enable ? "enabled" : "disabled");
		k_mutex_unlock(&cs_procedure_mutex);
		return 0;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = enable ? 1 : 0,
	};

	err = bt_le_cs_procedure_enable(connection, &params);
	if (err) {
		LOG_ERR("Failed to %s CS procedures (err %d)", 
			enable ? "enable" : "disable", err);
		k_mutex_unlock(&cs_procedure_mutex);
		return err;
	}

	LOG_INF("Channel Sounding procedures %s", enable ? "enabled" : "disabled");
	k_mutex_unlock(&cs_procedure_mutex);

	return 0;
}

bool channel_sounding_is_enabled(void)
{
	bool enabled;

	k_mutex_lock(&cs_procedure_mutex, K_FOREVER);
	enabled = cs_enabled;
	k_mutex_unlock(&cs_procedure_mutex);

	return enabled;
}

bool channel_sounding_get_ifft_distance(float *ifft_distance)
{
	if (!ifft_distance) {
		return false;
	}

	if (!cs_initialized || !cs_enabled) {
		LOG_DBG("Channel Sounding not initialized or not enabled");
		return false;
	}

	if (buffer_num_valid == 0) {
		LOG_DBG("No valid distance estimates available");
		return false;
	}

	/* Get distance estimates for antenna path 0 (first antenna) */
	cs_de_dist_estimates_t distance_on_ap0 = get_distance(0);

	/* Check if IFFT estimate is valid (finite number) */
	if (!isfinite(distance_on_ap0.ifft)) {
		LOG_DBG("IFFT distance estimate is not finite");
		return false;
	}

	*ifft_distance = distance_on_ap0.ifft;
	LOG_DBG("Retrieved IFFT distance: %.2f", (double)*ifft_distance);

	return true;
}

bool channel_sounding_is_known_device(const uint8_t *addr)
{
	if (!addr) {
		return false;
	}

	// Check if it matches Device 1
	if (memcmp(addr, known_device1_mac, 6) == 0) {
		LOG_DBG("Device matches known Device 1");
		return true;
	}

	// Check if it matches Device 2
	if (memcmp(addr, known_device2_mac, 6) == 0) {
		LOG_DBG("Device matches known Device 2");
		return true;
	}

	LOG_DBG("Unknown device: %02X:%02X:%02X:%02X:%02X:%02X",
		addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
	return false;
}

bool channel_sounding_get_remote_address(uint8_t *remote_addr)
{
	if (!remote_addr) {
		LOG_ERR("Invalid remote_addr pointer");
		return false;
	}

	if (!remote_address_valid) {
		LOG_DBG("No valid remote device address available");
		return false;
	}

	memcpy(remote_addr, remote_device_addr, 6);
	LOG_DBG("Retrieved remote device address: %02X:%02X:%02X:%02X:%02X:%02X",
		remote_addr[5], remote_addr[4], remote_addr[3],
		remote_addr[2], remote_addr[1], remote_addr[0]);
	
	return true;
}

