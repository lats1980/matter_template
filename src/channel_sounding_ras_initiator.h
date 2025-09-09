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
 * @brief Initialize the Channel Sounding RAS Initiator module
 * 
 * This function initializes the Channel Sounding functionality and starts
 * the dedicated thread that manages the Channel Sounding operations.
 * 
 * @return 0 on success, negative error code on failure
 */
int channel_sounding_init(void);

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
 * @brief Check if Channel Sounding is currently enabled
 * 
 * @return true if Channel Sounding is enabled, false otherwise
 */
bool channel_sounding_is_enabled(void);

/**
 * @brief Get the latest IFFT distance estimate from Channel Sounding
 * 
 * This function retrieves the most recent IFFT distance estimate from
 * the first antenna path. Used by RF sensing occupancy detection.
 * 
 * @param ifft_distance Pointer to store the IFFT distance value
 * @return true if valid IFFT distance is available, false otherwise
 */
bool channel_sounding_get_ifft_distance(float *ifft_distance);

#ifdef __cplusplus
}
#endif

