/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "occupancy_sensor_pir.h"
#include "app/task_executor.h"

#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>
#include <app/EventLogging.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "channel_sounding_ras_initiator.h"

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OccupancySensing;
using namespace chip::app::Clusters::OccupancySensing::Structs;
using namespace chip::DeviceLayer;

// GPIO device node from device tree
#define PIR_GPIO_NODE DT_NODELABEL(gpio0)

CHIP_ERROR OccupancySensorPIR::Init()
{
    int ret;

    if (mInitialized) {
        LOG_INF("PIR sensor already initialized");
        return CHIP_NO_ERROR;
    }

    LOG_INF("Initializing PIR occupancy sensor on P0.%d for nrf54L15DK", kPirPin);

    // Get GPIO device from device tree
    mGpioDevice = DEVICE_DT_GET(PIR_GPIO_NODE);
    if (!device_is_ready(mGpioDevice)) {
        LOG_ERR("GPIO device not ready");
        return CHIP_ERROR_INTERNAL;
    }

#if defined(CONFIG_SIMULATED_PIR_SENSOR)
    if (gpio_pin_configure(mGpioDevice, kPirPin, GPIO_INPUT | GPIO_PULL_UP) < 0) {
        LOG_ERR("Failed to configure PIR GPIO pin P0.%d", kPirPin);
        return CHIP_ERROR_INTERNAL;
    }
#else
    if (gpio_pin_configure(mGpioDevice, kPirPin, GPIO_INPUT | GPIO_PULL_DOWN) < 0) {
        LOG_ERR("Failed to configure PIR GPIO pin P0.%d", kPirPin);
        return CHIP_ERROR_INTERNAL;
    }
#endif

    // Initialize GPIO callback structure
    gpio_init_callback(&mGpioCallback, PirInterruptCallback, BIT(kPirPin));
    ret = gpio_add_callback(mGpioDevice, &mGpioCallback);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return CHIP_ERROR_INTERNAL;
    }

    // Enable interrupt on rising edge (PIR sensor activation)
    ret = gpio_pin_interrupt_configure(mGpioDevice, kPirPin, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
        return CHIP_ERROR_INTERNAL;
    }

    // Initialize work queue and timer for deferred processing
    k_work_init(&mPirWork, PirWorkHandler);

    // Initialize Matter OccupancySensing cluster instance
    static std::unique_ptr<OccupancySensing::Instance> occupancySensorInstance;
    occupancySensorInstance = std::make_unique<OccupancySensing::Instance>(BitMask<OccupancySensing::Feature, uint32_t>(OccupancySensing::Feature::kPassiveInfrared));

    mClusterInstance = occupancySensorInstance.get();
    
    CHIP_ERROR err = mClusterInstance->Init();
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to initialize OccupancySensing cluster: %" CHIP_ERROR_FORMAT, err.Format());
        return err;
    }

    mInitialized = true;
    LOG_INF("PIR occupancy sensor initialized successfully on Endpoint %d", kOccupancySensorEndpointId);

    return CHIP_NO_ERROR;
}

CHIP_ERROR OccupancySensorPIR::SetOccupancyState(bool occupied)
{
    if (!mInitialized) {
        LOG_ERR("PIR sensor not initialized");
        return CHIP_ERROR_INCORRECT_STATE;
    }

    if (occupied)
    {
        uint16_t * holdTime = chip::app::Clusters::OccupancySensing::GetHoldTimeForEndpoint(kOccupancySensorEndpointId);
        if (holdTime != nullptr)
        {
            CHIP_ERROR err = chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds16(*holdTime), OccupancySensorPIR::OccupancyPresentTimerHandler,
                reinterpret_cast<void *>(static_cast<uintptr_t>(kOccupancySensorEndpointId)));
            LOG_INF("Start HoldTime timer");
            if (CHIP_NO_ERROR != err)
            {
                LOG_INF("Failed to start HoldTime timer.");
            }
        }
    }
    chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
    Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(kOccupancySensorEndpointId, &currentOccupancy);
    VerifyOrDie(status == Protocols::InteractionModel::Status::Success);

    if (static_cast<BitMask<chip::app::Clusters::OccupancySensing::OccupancyBitmap>>(occupied) == currentOccupancy) {
        // No state change needed
        return CHIP_NO_ERROR;
    }
    LOG_INF("PIR sensor state change: UNOCCUPIED -> OCCUPIED");

    status = chip::app::Clusters::OccupancySensing::Attributes::Occupancy::Set(
        kOccupancySensorEndpointId, (uint8_t)occupied);
    if (status != Protocols::InteractionModel::Status::Success)
    {
        LOG_ERR("Failed to set occupancy attribute: %d", static_cast<int>(status));
        return CHIP_ERROR_INTERNAL;
    }

    // Send OccupancyChanged event
    chip::app::Clusters::OccupancySensing::Events::OccupancyChanged::Type event;
    event.occupancy = occupied ? chip::app::Clusters::OccupancySensing::OccupancyBitmap::kOccupied : static_cast<chip::app::Clusters::OccupancySensing::OccupancyBitmap>(0);

    EventNumber eventNumber;
    CHIP_ERROR err = LogEvent(event, kOccupancySensorEndpointId, eventNumber);
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to log occupancy event: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        LOG_INF("OccupancyChanged event sent (EventNumber: %" PRIu64 ")", eventNumber);
    }

    return CHIP_NO_ERROR;
}

bool OccupancySensorPIR::IsOccupied()
{
    chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
    Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(kOccupancySensorEndpointId, &currentOccupancy);
    VerifyOrDie(status == Protocols::InteractionModel::Status::Success);
    return currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied);
}

void OccupancySensorPIR::OccupancyPresentTimerHandler(System::Layer * systemLayer, void * appState)
{
    EndpointId endpointId = static_cast<EndpointId>(reinterpret_cast<uintptr_t>(appState));
    chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;

    Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(endpointId, &currentOccupancy);
    VerifyOrDie(status == Protocols::InteractionModel::Status::Success);

    uint8_t clearValue = 0;
    if (!currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied))
    {
        return;
    }

    status = OccupancySensing::Attributes::Occupancy::Set(endpointId, clearValue);
    if (status != Protocols::InteractionModel::Status::Success)
    {
        LOG_ERR("Failed to set occupancy state.");
    }
    else
    {
        LOG_ERR("Set Occupancy attribute to clear");
    }

    // Send OccupancyChanged event
    chip::app::Clusters::OccupancySensing::Events::OccupancyChanged::Type event;
    event.occupancy = clearValue ? chip::app::Clusters::OccupancySensing::OccupancyBitmap::kOccupied : static_cast<chip::app::Clusters::OccupancySensing::OccupancyBitmap>(0);

    EventNumber eventNumber;
    CHIP_ERROR err = LogEvent(event, kOccupancySensorEndpointId, eventNumber);
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to log occupancy event: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        LOG_INF("OccupancyChanged event sent (EventNumber: %" PRIu64 ")", eventNumber);
    }
    LOG_INF("PIR sensor state change: OCCUPIED -> UNOCCUPIED");
}

void OccupancySensorPIR::PirInterruptCallback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    
    // Get the sensor instance from the callback structure
    OccupancySensorPIR *sensor = CONTAINER_OF(cb, OccupancySensorPIR, mGpioCallback);
    
    // Schedule work to handle the PIR sensor activation in a different context
    // This is important to avoid doing too much work in the interrupt context
    k_work_submit(&sensor->mPirWork);
}

void OccupancySensorPIR::PirWorkHandler(k_work *work)
{
    // Get the sensor instance from the work structure
    OccupancySensorPIR *sensor = CONTAINER_OF(work, OccupancySensorPIR, mPirWork);
    // Read the current GPIO state to confirm the interrupt was valid
    int pinValue = gpio_pin_get(sensor->mGpioDevice, kPirPin);
    LOG_INF("GPIO pin value: %d", pinValue);
    if (pinValue > 0) {
        // PIR sensor detected motion - set occupied state
        Nrf::PostTask([sensor] { 
            CHIP_ERROR err = sensor->SetOccupancyState(true);
            if (err != CHIP_NO_ERROR) {
                LOG_ERR("Failed to set occupied state: %" CHIP_ERROR_FORMAT, err.Format());
            }
        });
    }
}
