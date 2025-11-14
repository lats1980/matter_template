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
 * @brief RFS Operating modes
 */
typedef enum {
    RFS_MODE_NORMAL = 0,     /**< Normal RF sensing mode */
    RFS_MODE_LOW_POWER = 1,  /**< Low power RF sensing mode */
    RFS_MODE_STOPPED = 2     /**< RF sensing stopped */
} rfs_mode_t;

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
     * @brief Get current RFS operating mode
     * 
     * @return Current RFS mode
     */
    rfs_mode_t GetRFSMode() const { return mCurrentMode; }

    /**
     * @brief Set RFS operating mode
     * 
     * @param mode New mode to set
     */
    void SetRFSMode(rfs_mode_t mode);

    /**
     * @brief Toggle RFS mode (cycles through normal -> low power -> stopped -> normal)
     */
    void ToggleRFSMode();

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
    static constexpr uint32_t kRFSensingHoldTimeNormal = 3 * CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;
    static constexpr uint32_t kRFSensingHoldTimeLowPower = 9 * CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;
    static constexpr chip::EndpointId kRfsFirstEndpoint = 1;
    static constexpr chip::EndpointId kRfsLastEndpoint = kRfsFirstEndpoint + CONFIG_BT_MAX_PAIRED - 1;
    
    // Zephyr resources
    struct k_timer mModeIndicatorTimer;  // Timer for LED blinking in low power mode
    
    // State tracking
    mutable chip::EndpointId mConnectedDevice = kInvalidEndpointId; // Cache for connected device
    rfs_mode_t mCurrentMode = RFS_MODE_NORMAL;  // Current RFS operating mode
    bool mLedBlinkState = false;  // LED blink state for low power mode
    uint32_t kRfSensingOperationIntervalMs; // next operation interval based on mode
};
