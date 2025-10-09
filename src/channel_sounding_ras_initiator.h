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

/**
 * @brief Initialize the Channel Sounding RAS Initiator module
 * 
 * This function initializes the Channel Sounding functionality and starts
 * the dedicated thread that manages the Channel Sounding operations.
 * 
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_init(void);

/**
 * @brief Get the current Channel Sounding state
 * 
 * @return channel_sounding_state enum value representing the current state
 */
enum channel_sounding_state get_channel_sounding_state(void);

/**
 * @brief Set the Channel Sounding state (for internal use)
 * 
 * This function sets the current state of the Channel Sounding module.
 * It is intended for internal use within the module only.
 * 
 * @param new_state The new state to set
 * @return true if state was changed, false if it was not permitted
 */
bool set_channel_sounding_state(enum channel_sounding_state new_state);

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
 * @brief Get the latest distance estimate from Channel Sounding
 * 
 * This function retrieves the most recent distance estimate from
 * the antenna path. Used by RF sensing occupancy detection.
 * 
 * @param distance Pointer to store the distance value
 * @return true if valid distance is available, false otherwise
 */
bool channel_sounding_get_distance(float *distance);

/**
 * @brief Get the remote connected device's Bluetooth address
 * 
 * This function retrieves the Bluetooth address of the currently connected
 * remote device. Used for device identification in occupancy sensing.
 * 
 * @param remote_addr Pointer to store the remote device address (6 bytes)
 * @return true if a device is connected and address is available, false otherwise
 */
bool channel_sounding_get_remote_address(uint8_t *remote_addr);

/**
 * @brief Check if a Bluetooth address is a known RF sensing device
 * 
 * This function checks if the given MAC address matches any of the configured
 * RF sensing devices (Device 1 or Device 2).
 * 
 * @param addr Bluetooth address to check (6 bytes)
 * @return true if the address matches a known device, false otherwise
 */
bool channel_sounding_is_known_device(const uint8_t *addr);

#ifdef __cplusplus
}
#endif

