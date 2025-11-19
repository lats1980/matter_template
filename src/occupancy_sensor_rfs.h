/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "occupancy_sensor_base.h"
#include <zephyr/kernel.h>

using namespace chip;

/**
 * @brief RF Sensing Occupancy Sensor implementation for Matter
 * 
 * This class implements an RF Sensing occupancy sensor for Matter using
 * Channel Sounding distance data for occupancy detection, inheriting common
 * functionality from OccupancySensorBase.
 */
class OccupancySensorRFS : public OccupancySensorBase {
public:
    /**
     * @brief Get the singleton instance
     */
    static OccupancySensorRFS &Instance()
    {
        static OccupancySensorRFS sInstance;
        return sInstance;
    }

    /**
     * @brief Initialize the RF Sensing occupancy sensor
     * 
     * Initializes the Matter OccupancySensing cluster with RF sensing capabilities.
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR Init() override;

    /**
     * @brief Validate if the given endpoint ID is valid for this sensor
     * 
     * Implementation of pure virtual method from base class
     * 
     * @param endpointId The endpoint ID to validate
     * @return true if valid, false otherwise
     */
    bool IsValidEndpoint(chip::EndpointId endpointId) const override
    {
        return (endpointId >= kRfsFirstEndpoint) && (endpointId <= kRfsLastEndpoint);
    }

    /**
     * @brief Check if a specific RF Sensing endpoint is occupied
     * 
     * @param endpointId The endpoint ID to check (optional, checks all if not provided)
     * @return true if the endpoint is occupied, false otherwise
     */
    bool IsOccupied(chip::EndpointId endpointId = kInvalidEndpointId) const;

    /**
     * @brief Set the hold time for occupancy state
     * 
     * @param holdTimeSec Hold time in seconds
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR SetHoldTime(uint16_t holdTimeSec);

    /**
     * @brief Handle button press events for mode switching
     * 
     * @param button_pressed true if button is pressed, false if released
     */
    void HandleButtonEvent(bool button_pressed);

private:
    OccupancySensorRFS() = default;
    ~OccupancySensorRFS() = default;

    // Non-copyable
    OccupancySensorRFS(const OccupancySensorRFS&) = delete;
    OccupancySensorRFS& operator=(const OccupancySensorRFS&) = delete;

    static void RfsEventHandler(uint8_t bond_idx, float distance);

    /**
     * @brief Update LED1 based on current RFS mode
     */
    void UpdateModeIndicatorLED();

    /**
     * @brief Timer callback for LED blinking in low power mode
     */
    static void ModeIndicatorTimerHandler(k_timer *timer);

    bool remote_address_valid = false;
    
    // Configuration constants
    static constexpr float kOccupancyThreshold = CONFIG_RFS_OCCUPANCY_THRESHOLD; // threshold indicates occupancy
    static constexpr chip::EndpointId kRfsFirstEndpoint = 1;
    static constexpr chip::EndpointId kRfsLastEndpoint = kRfsFirstEndpoint + CONFIG_BT_MAX_PAIRED - 1;
    
    // Zephyr resources
    struct k_timer mModeIndicatorTimer;  // Timer for LED blinking in low power mode
    
    // State tracking
    mutable chip::EndpointId mConnectedDevice = kInvalidEndpointId; // Cache for connected device
    bool mLedBlinkState = false;  // LED blink state for low power mode
    uint32_t kRfSensingOperationIntervalMs; // next operation interval based on mode
    bool mPreemptiveMode = false; // preemptive mode flag
};
