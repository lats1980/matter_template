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

    // Initialize Matter OccupancySensing cluster instance with RF Sensing feature
    CHIP_ERROR err = InitializeClusterInstance(BitMask<OccupancySensing::Feature, uint32_t>(OccupancySensing::Feature::kRFSensing));
    if (err != CHIP_NO_ERROR) {
        return err;
    }
    
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
    ARG_UNUSED(work);
    
    // Get the sensor instance using singleton pattern
    OccupancySensorRFS *sensor = &OccupancySensorRFS::Instance();
    
    LOG_DBG("RF Sensing work handler triggered");
    
    // Check RF sensing data and determine occupancy
    bool occupied = sensor->CheckRFSensing();
    
    // Update occupancy state if occupied
    if (occupied) {
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

