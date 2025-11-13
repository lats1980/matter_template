/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "occupancy_sensor_base.h"
#include <zephyr/drivers/gpio.h>

/**
 * @brief PIR Occupancy Sensor implementation for Matter
 * 
 * This class implements a PIR (Passive Infrared) occupancy sensor for Matter
 * inheriting common functionality from OccupancySensorBase
 */
class OccupancySensorPIR : public OccupancySensorBase {
public:
    /**
     * @brief Get the singleton instance
     */
    static OccupancySensorPIR &Instance()
    {
        static OccupancySensorPIR sInstance;
        return sInstance;
    }

    /**
     * @brief Initialize the PIR occupancy sensor
     * 
     * Sets up GPIO as input with interrupt for PIR sensor detection
     * and initializes the Matter OccupancySensing cluster on Endpoint 1.
     * 
     * @return CHIP_ERROR CHIP_NO_ERROR on success, error code otherwise
     */
    CHIP_ERROR Init() override;

    /**
     * @brief Get the endpoint ID for the occupancy sensor
     * 
     * @return chip::EndpointId The endpoint ID
     */
    chip::EndpointId GetEndpointId() const override { return kPirEndpoint; }

private:
    OccupancySensorPIR() = default;
    ~OccupancySensorPIR() = default;

    // Non-copyable
    OccupancySensorPIR(const OccupancySensorPIR&) = delete;
    OccupancySensorPIR& operator=(const OccupancySensorPIR&) = delete;

    /**
     * @brief GPIO interrupt callback for PIR sensor
     * 
     * Called when PIR sensor detects motion (rising edge)
     */
    static void PirInterruptCallback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    /**
     * @brief Work handler for PIR sensor state changes
     * 
     * Handles the actual occupancy state update in the system work queue context
     */
    static void PirWorkHandler(k_work *work);


    // Configuration constants
    static constexpr chip::EndpointId kPirEndpoint = 1 + CONFIG_BT_MAX_PAIRED;
#if defined(CONFIG_SIMULATED_PIR_SENSOR)
    static constexpr uint8_t kPirPin = 4; // P0.04 (simulated with button)
#else
    static constexpr uint8_t kPirPin = 3; // P0.03
#endif
    
    // GPIO and Zephyr resources
    const struct device *mGpioDevice = nullptr;
    struct gpio_callback mGpioCallback;
    struct k_work mPirWork;
};
