/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "occupancy_sensor_rfs.h"
#include "channel_sounding_ras_initiator.h"
#include "app/task_executor.h"
#include "app/matter_init.h"

#include "lib/support/CodeUtils.h"

#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>
#include <app/EventLogging.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OccupancySensing;
using namespace chip::app::Clusters::OccupancySensing::Structs;
using namespace chip::DeviceLayer;

void OccupancySensorRFS::MatterEventHandler(const ChipDeviceEvent *event, intptr_t /* unused */)
{
	switch (event->Type) {
        case DeviceEventType::kThreadStateChange:
            if(ConnectivityMgrImpl().IsIPv6NetworkProvisioned() &&
                        ConnectivityMgrImpl().IsIPv6NetworkEnabled()) {
                LOG_INF("Thread is provisioned and enabled - starting Channel Sounding procedure");
                /* Initialize Channel Sounding */
                int err = channel_sounding_init();
                if (err) {
                    LOG_ERR("Channel Sounding initialization failed (err %d)", err);
                }
            }
            break;
	default:
		break;
	}
}

CHIP_ERROR OccupancySensorRFS::Init()
{
    if (mInitialized) {
        LOG_INF("RF Sensing occupancy sensor already initialized");
        return CHIP_NO_ERROR;
    }

    LOG_INF("Initializing RF Sensing occupancy sensor for Endpoint %d", kOccupancySensorEndpointId);

    // Initialize work queue and timers for deferred processing
    k_work_init(&mRfsWork, RfsWorkHandler);
    k_timer_init(&mRfsTimer, RfsTimerCallback, nullptr);

    // Initialize Matter OccupancySensing cluster instance
    static std::unique_ptr<OccupancySensing::Instance> occupancySensorInstance;
    occupancySensorInstance = std::make_unique<OccupancySensing::Instance>(BitMask<OccupancySensing::Feature, uint32_t>(OccupancySensing::Feature::kRFSensing));
    mClusterInstance = occupancySensorInstance.get();
    ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(MatterEventHandler, 0));

    mInitialized = true;
    LOG_INF("RF Sensing occupancy sensor initialized successfully on Endpoint %d", kOccupancySensorEndpointId);

    return CHIP_NO_ERROR;
}

CHIP_ERROR OccupancySensorRFS::StartRFSensing()
{
    if (!mInitialized) {
        LOG_ERR("RF Sensing sensor not initialized");
        return CHIP_ERROR_INCORRECT_STATE;
    }

    if (mRfSensingActive) {
        LOG_INF("RF Sensing monitoring already active");
        return CHIP_NO_ERROR;
    }

    // Start periodic timer for RF sensing checks
    k_timer_start(&mRfsTimer, K_MSEC(kRfSensingIntervalMs), K_MSEC(kRfSensingIntervalMs));
    mRfSensingActive = true;

    LOG_INF("RF Sensing monitoring started (interval: %d ms, threshold: %.1f)", 
            kRfSensingIntervalMs, (double)kIfftOccupancyThreshold);

    return CHIP_NO_ERROR;
}

void OccupancySensorRFS::StopRFSensing()
{
    if (!mRfSensingActive) {
        return;
    }

    k_timer_stop(&mRfsTimer);
    k_timer_stop(&mUnoccupiedTimer);
    mRfSensingActive = false;

    LOG_INF("RF Sensing monitoring stopped");
}

CHIP_ERROR OccupancySensorRFS::SetOccupancyState(bool occupied)
{
    if (!mInitialized) {
        LOG_ERR("RF sensing not initialized");
        return CHIP_ERROR_INCORRECT_STATE;
    }

    if (occupied)
    {
        uint16_t * holdTime = chip::app::Clusters::OccupancySensing::GetHoldTimeForEndpoint(kOccupancySensorEndpointId);
        if (holdTime != nullptr)
        {
            CHIP_ERROR err = chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds16(*holdTime), OccupancySensorRFS::OccupancyPresentTimerHandler,
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
    LOG_INF("RF sensing state change: UNOCCUPIED -> OCCUPIED");

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

bool OccupancySensorRFS::CheckRFSensing()
{
    float ifft_distance = 0.0f;
    
    // Get IFFT distance from Channel Sounding
    if (!channel_sounding_get_ifft_distance(&ifft_distance)) {
        LOG_DBG("Failed to get IFFT distance from Channel Sounding");
        return false; // No valid data, assume unoccupied
    }

    // Check if IFFT indicates occupancy
    bool occupied = (ifft_distance < kIfftOccupancyThreshold);
    
    LOG_DBG("RF Sensing check: IFFT=%.2f, threshold=%.1f, occupied=%s", 
            (double)ifft_distance, (double)kIfftOccupancyThreshold, occupied ? "YES" : "NO");
    
    return occupied;
}

void OccupancySensorRFS::RfsWorkHandler(k_work *work)
{
    // Get the sensor instance from the work structure
    OccupancySensorRFS *sensor = CONTAINER_OF(work, OccupancySensorRFS, mRfsWork);
    
    LOG_DBG("RF Sensing work handler triggered");
    
    // Check RF sensing data and determine occupancy
    bool occupied = sensor->CheckRFSensing();
    
    // Update occupancy state if there's a change
    if (occupied != sensor->mOccupied) {
        Nrf::PostTask([sensor, occupied] { 
            CHIP_ERROR err = sensor->SetOccupancyState(occupied);
            if (err != CHIP_NO_ERROR) {
                LOG_ERR("Failed to set RF Sensing occupancy state: %" CHIP_ERROR_FORMAT, err.Format());
            }
        });
    }
}

void OccupancySensorRFS::RfsTimerCallback(k_timer *timer)
{
    ARG_UNUSED(timer);
    
    // Get the sensor instance (singleton)
    OccupancySensorRFS *sensor = &OccupancySensorRFS::Instance();
    
    // Schedule work to check RF sensing data
    k_work_submit(&sensor->mRfsWork);
}

void OccupancySensorRFS::OccupancyPresentTimerHandler(System::Layer * systemLayer, void * appState)
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
    LOG_INF("RF sensing state change: OCCUPIED -> UNOCCUPIED");
}
