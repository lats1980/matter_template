/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <platform/CHIPDeviceLayer.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>
#include <zephyr/kernel.h>

/**
 * @brief Base class for occupancy sensors in Matter
 * 
 * This abstract base class provides common functionality for different
 * types of occupancy sensors (PIR, RF Sensing, etc.) including:
 * - SetOccupancyState() method
 * - IsOccupied() method
 * - GetEndpointId() method
 * - Common timer handling for occupancy timeout
 * - Matter cluster instance management
 */
class OccupancySensorBase {
public:
    virtual ~OccupancySensorBase() = default;

    /**
     * @brief Initialize the occupancy sensor
     * 
     * Pure virtual method that must be implemented by derived classes
     * to handle sensor-specific initialization
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    virtual CHIP_ERROR Init() = 0;

    /**
     * @brief Set the occupancy state
     * 
     * Updates the occupancy attribute and sends OccupancyChanged events
     * Common implementation for all sensor types
     * 
     * @param occupied true if occupied, false if unoccupied
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR SetOccupancyState(bool occupied);

    /**
     * @brief Get the current occupancy state
     * 
     * Virtual method that can be overridden by derived classes if they
     * need custom state tracking (default queries Matter attribute directly)
     * 
     * @return true if occupied, false if unoccupied
     */
    virtual bool IsOccupied();

    /**
     * @brief Get the endpoint ID for the occupancy sensor
     * 
     * Pure virtual method that must be implemented by derived classes
     * 
     * @return chip::EndpointId The endpoint ID
     */
    virtual chip::EndpointId GetEndpointId() const = 0;

protected:
    /**
     * @brief Constructor for derived classes
     */
    OccupancySensorBase() = default;

    /**
     * @brief Common timer callback for occupancy timeout
     * 
     * Static method that handles occupancy timeout for all sensor types
     * Called when the occupancy timeout expires to set state to unoccupied
     */
    static void OccupancyPresentTimerHandler(chip::System::Layer * systemLayer, void * appState);

    /**
     * @brief Initialize the Matter OccupancySensing cluster
     * 
     * Common helper method to initialize the cluster instance with specified features
     * 
     * @param features BitMask of OccupancySensing features to enable
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR InitializeClusterInstance(chip::BitMask<chip::app::Clusters::OccupancySensing::Feature, uint32_t> features);

    // State tracking
    bool mInitialized = false;
    
    // Matter cluster instance
    chip::app::Clusters::OccupancySensing::Instance *mClusterInstance = nullptr;

private:
    // Non-copyable
    OccupancySensorBase(const OccupancySensorBase&) = delete;
    OccupancySensorBase& operator=(const OccupancySensorBase&) = delete;
};
