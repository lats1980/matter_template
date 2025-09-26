/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "occupancy_sensor_base.h"
#include "app/task_executor.h"

#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>
#include <app/EventLogging.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OccupancySensing;
using namespace chip::app::Clusters::OccupancySensing::Structs;
using namespace chip::DeviceLayer;

CHIP_ERROR OccupancySensorBase::SetOccupancyState(bool occupied)
{
    if (!mInitialized) {
        LOG_ERR("Occupancy sensor not initialized");
        return CHIP_ERROR_INCORRECT_STATE;
    }

    chip::EndpointId endpointId = GetEndpointId();

    if (occupied)
    {
        uint16_t * holdTime = chip::app::Clusters::OccupancySensing::GetHoldTimeForEndpoint(endpointId);
        if (holdTime != nullptr)
        {
            CHIP_ERROR err = chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds16(*holdTime), OccupancySensorBase::OccupancyPresentTimerHandler,
                reinterpret_cast<void *>(static_cast<uintptr_t>(endpointId)));
            LOG_INF("Start HoldTime timer");
            if (CHIP_NO_ERROR != err)
            {
                LOG_INF("Failed to start HoldTime timer.");
            }
        }
    }

    chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
    Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(endpointId, &currentOccupancy);
    VerifyOrDie(status == Protocols::InteractionModel::Status::Success);

    if (static_cast<BitMask<chip::app::Clusters::OccupancySensing::OccupancyBitmap>>(occupied) == currentOccupancy) {
        // No state change needed
        return CHIP_NO_ERROR;
    }

    LOG_INF("Occupancy sensor state change: %s -> %s", 
            currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied) ? "OCCUPIED" : "UNOCCUPIED",
            occupied ? "OCCUPIED" : "UNOCCUPIED");

    status = chip::app::Clusters::OccupancySensing::Attributes::Occupancy::Set(
        endpointId, (uint8_t)occupied);
    if (status != Protocols::InteractionModel::Status::Success)
    {
        LOG_ERR("Failed to set occupancy attribute: %d", static_cast<int>(status));
        return CHIP_ERROR_INTERNAL;
    }

    // Send OccupancyChanged event
    chip::app::Clusters::OccupancySensing::Events::OccupancyChanged::Type event;
    event.occupancy = occupied ? chip::app::Clusters::OccupancySensing::OccupancyBitmap::kOccupied : static_cast<chip::app::Clusters::OccupancySensing::OccupancyBitmap>(0);

    EventNumber eventNumber;
    CHIP_ERROR err = LogEvent(event, endpointId, eventNumber);
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to log occupancy event: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        LOG_INF("OccupancyChanged event sent (EventNumber: %" PRIu64 ")", eventNumber);
    }

    return CHIP_NO_ERROR;
}

bool OccupancySensorBase::IsOccupied()
{
    chip::EndpointId endpointId = GetEndpointId();
    chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
    Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(endpointId, &currentOccupancy);
    VerifyOrDie(status == Protocols::InteractionModel::Status::Success);
    return currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied);
}

void OccupancySensorBase::OccupancyPresentTimerHandler(System::Layer * systemLayer, void * appState)
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
        LOG_INF("Set Occupancy attribute to clear");
    }

    // Send OccupancyChanged event
    chip::app::Clusters::OccupancySensing::Events::OccupancyChanged::Type event;
    event.occupancy = clearValue ? chip::app::Clusters::OccupancySensing::OccupancyBitmap::kOccupied : static_cast<chip::app::Clusters::OccupancySensing::OccupancyBitmap>(0);

    EventNumber eventNumber;
    CHIP_ERROR err = LogEvent(event, endpointId, eventNumber);
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to log occupancy event: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        LOG_INF("OccupancyChanged event sent (EventNumber: %" PRIu64 ")", eventNumber);
    }
    LOG_INF("Occupancy sensor state change: OCCUPIED -> UNOCCUPIED");
}

CHIP_ERROR OccupancySensorBase::InitializeClusterInstance(chip::BitMask<chip::app::Clusters::OccupancySensing::Feature, uint32_t> features)
{
    static std::unique_ptr<OccupancySensing::Instance> occupancySensorInstance;
    occupancySensorInstance = std::make_unique<OccupancySensing::Instance>(features);

    mClusterInstance = occupancySensorInstance.get();
    
    CHIP_ERROR err = mClusterInstance->Init();
    if (err != CHIP_NO_ERROR) {
        LOG_ERR("Failed to initialize OccupancySensing cluster: %" CHIP_ERROR_FORMAT, err.Format());
        return err;
    }

    return CHIP_NO_ERROR;
}
