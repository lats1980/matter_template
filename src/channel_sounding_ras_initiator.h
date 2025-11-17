/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enum representing the Channel Sounding state
 */
enum channel_sounding_state {
    CS_STATE_UNINITIALIZED = -1,
    CS_STATE_STOPPED,
    CS_STATE_CONNECTING,
    CS_STATE_STARTED
};

typedef void (*channel_sounding_event_handler_t)(uint8_t index, float distance);

/**
 * @brief Initialize the Channel Sounding RAS Initiator module
 * 
 * This function initializes the Channel Sounding functionality and starts
 * the dedicated thread that manages the Channel Sounding operations.
 * 
 * @param event_handler Callback function to handle Channel Sounding events
 * 
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_init(channel_sounding_event_handler_t event_handler);

/**
 * @brief Get the current Channel Sounding state
 * 
 * @return channel_sounding_state enum value representing the current state
 */
enum channel_sounding_state get_channel_sounding_state(void);

/**
 * @brief Enable or disable Channel Sounding procedures
 * 
 * This function provides a wrapper around bt_le_cs_procedure_enable to
 * allow other modules to control Channel Sounding functionality.
 * 
 * @param enable true to enable Channel Sounding, false to disable
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_procedure_enable(bool enable);

/**
 * @brief Set the inactive interval between Channel Sounding procedures
 * 
 * This function sets the time interval (in milliseconds) between
 * consecutive Channel Sounding procedures when they are enabled.
 * 
 * @param interval_ms Inactive interval in milliseconds
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_set_inactive_interval(uint32_t interval_ms);

/**
 * @brief Enable or disable preemptive mode for Channel Sounding
 * 
 * In preemptive mode, Channel Sounding procedures reconnect to
 * last known devices with higher priority. In non-preemptive mode,
 * devices are scanned and connected in the order they were last paired.
 * 
 * @param enable true to enable preemptive mode, false to disable
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_set_preemptive_mode(bool enable);

/**
 * @brief Check if Channel Sounding is in preemptive mode
 * 
 * @return true if preemptive mode is enabled, false otherwise
 */
bool channel_sounding_is_preemptive_mode();

#ifdef __cplusplus
}
#endif

