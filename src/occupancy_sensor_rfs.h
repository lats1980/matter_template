/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "occupancy_sensor_base.h"
#include <zephyr/kernel.h>

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
     * Initializes the Matter OccupancySensing cluster on Endpoint 2 with
     * RF sensing capabilities and starts the periodic IFFT monitoring.
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR Init() override;

    /**
     * @brief Get the endpoint ID for the occupancy sensor
     * 
     * @return chip::EndpointId The endpoint ID
     */
    chip::EndpointId GetEndpointId() const override { return kOccupancySensorEndpointId; }

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
     * @brief Timer callback for periodic RF sensing checks
     * 
     * Schedules work to check IFFT values at regular intervals
     */
    static void RfsTimerCallback(k_timer *timer);


    /**
     * @brief Check IFFT values and determine occupancy
     * 
     * Gets IFFT values from Channel Sounding and determines if space is occupied
     * 
     * @return true if IFFT indicates occupancy, false otherwise
     */
    bool CheckRFSensing();

    // Configuration constants
    static constexpr chip::EndpointId kOccupancySensorEndpointId = 2;
    static constexpr float kIfftOccupancyThreshold = 3.0f; // IFFT < 3.0 indicates occupancy
    static constexpr uint32_t kRfSensingIntervalMs = 3000; // Check IFFT every 1 second
    
    // Zephyr resources
    struct k_work mRfsWork;
    struct k_timer mRfsTimer;
    struct k_timer mUnoccupiedTimer;
    
    // State tracking
    bool mRfSensingActive = false;
};
