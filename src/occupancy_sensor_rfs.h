/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "occupancy_sensor_base.h"
#include <zephyr/kernel.h>

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
 * Channel Sounding IFFT data for occupancy detection, inheriting common
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
     * Uses endpoint 2 for Device 1 (DC:B8:19:90:68:51) or endpoint 3 for Device 2 (DC:B8:19:90:68:52).
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR Init() override;

    /**
     * @brief Get the endpoint ID for the occupancy sensor
     * 
     * Returns endpoint 2 for device 1, endpoint 3 for device 2
     * 
     * @return chip::EndpointId The endpoint ID based on connected device
     */
    chip::EndpointId GetEndpointId() const override;

    /**
     * @brief Start RF sensing monitoring
     * 
     * Begins periodic monitoring of Channel Sounding IFFT values
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR StartRFSensing();

    /**
     * @brief Stop RF sensing monitoring
     * 
     * Stops periodic monitoring of Channel Sounding IFFT values
     */
    void StopRFSensing();

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

    /**
     * @brief Matter event handler
     * 
     * Handles Matter stack events relevant to the RFS occupancy sensor
     */
    static void MatterEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t data);

    /**
     * @brief Work handler for RF sensing monitoring
     * 
     * Periodically checks Channel Sounding IFFT values and updates occupancy state
     */
    static void RfsWorkHandler(k_work *work);


    /**
     * @brief Check IFFT values and determine occupancy
     * 
     * Gets IFFT values from Channel Sounding and determines if space is occupied
     * 
     * @return true if IFFT indicates occupancy, false otherwise
     */
    bool CheckRFSensing();

    /**
     * @brief Detect which device is connected
     * 
     * Determines the connected device based on remote MAC address from channel sounding
     * 
     * @return Device number (1 or 2), 0 if unknown
     */
    uint8_t DetectConnectedDevice() const;

    /**
     * @brief Reset device detection cache
     * 
     * Forces re-detection of the connected device on next call to DetectConnectedDevice()
     * Useful when device reconnects or changes
     */
    void ResetDeviceCache() { mConnectedDevice = 0; }

    /**
     * @brief Update LED1 based on current RFS mode
     */
    void UpdateModeIndicatorLED();

    /**
     * @brief Timer callback for LED blinking in low power mode
     */
    static void ModeIndicatorTimerHandler(k_timer *timer);

    // Device identification constants - configured via Kconfig
    static constexpr uint8_t kDeviceMac1[6] = {
        (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 40) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 32) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 24) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 16) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE1_MAC_ADDR >> 8) & 0xFF),
        (uint8_t)(CONFIG_RFS_DEVICE1_MAC_ADDR & 0xFF)
    }; // Device 1 MAC from Kconfig

    static constexpr uint8_t kDeviceMac2[6] = {
        (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 40) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 32) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 24) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 16) & 0xFF),
        (uint8_t)((CONFIG_RFS_DEVICE2_MAC_ADDR >> 8) & 0xFF),
        (uint8_t)(CONFIG_RFS_DEVICE2_MAC_ADDR & 0xFF)
    }; // Device 2 MAC from Kconfig
    
    // Configuration constants
    static constexpr chip::EndpointId kInvalidEndpointId = 0; // Invalid endpoint ID
    static constexpr chip::EndpointId kDevice1EndpointId = 2;  // Endpoint for device 1
    static constexpr chip::EndpointId kDevice2EndpointId = 3;  // Endpoint for device 2
    static constexpr float kIfftOccupancyThreshold = 3.0f; // IFFT < 3.0 indicates occupancy
    static constexpr uint32_t kRfSensingIntervalMs = 3000; // Check IFFT every 3 seconds
    static constexpr uint32_t kRfSensingSlowIntervalMs = 15000; // Check IFFT every 30 seconds
    static constexpr uint32_t kRFSensingHoldTimeNoemal = 3 * CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;
    static constexpr uint32_t kRFSensingHoldTimeLowPower = 9 * CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;
    
    // Zephyr resources
    struct k_work_delayable mRfsWork;
    struct k_timer mModeIndicatorTimer;  // Timer for LED blinking in low power mode
    
    // State tracking
    bool mRfSensingActive = false;
    mutable uint8_t mConnectedDevice = 0; // Cache for connected device (1, 2, or 0 for unknown)
    rfs_mode_t mCurrentMode = RFS_MODE_NORMAL;  // Current RFS operating mode
    bool mLedBlinkState = false;  // LED blink state for low power mode
};
