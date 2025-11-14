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
#include <app/util/endpoint-config-api.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>
#include <app/EventLogging.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <math.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OccupancySensing;
using namespace chip::app::Clusters::OccupancySensing::Structs;
using namespace chip::DeviceLayer;

void OccupancySensorRFS::RfsEventHandler(uint8_t bond_idx, float distance)
{
    LOG_DBG("Channel Sounding event received");

    if (distance < kOccupancyThreshold) {
        LOG_DBG("Distance %.2f below threshold %.1f - setting occupied", (double)distance, (double)kOccupancyThreshold);

        if (bond_idx >=0 && bond_idx < CONFIG_BT_MAX_PAIRED) {
            LOG_DBG("Confirmed connection to known RF sensing device");
            OccupancySensorRFS::Instance().remote_address_valid = true;
            // Map bond index to endpoint ID, skip endpoint 0 (root endpoint)
            OccupancySensorRFS::Instance().mConnectedDevice = bond_idx + 1;
        } else {
            OccupancySensorRFS::Instance().remote_address_valid = false;
            LOG_ERR("Failed to get remote device address");
            return;
        }

        Nrf::PostTask([] {
            CHIP_ERROR err = OccupancySensorRFS::Instance().SetOccupancyState(true, OccupancySensorRFS::Instance().mConnectedDevice);
            if (err != CHIP_NO_ERROR) {
                LOG_ERR("Failed to set RF Sensing occupancy state: %" CHIP_ERROR_FORMAT, err.Format());
            }
        });
    }
}

CHIP_ERROR OccupancySensorRFS::Init()
{
    if (mInitialized) {
        LOG_INF("RF Sensing occupancy sensor already initialized");
        return CHIP_NO_ERROR;
    }
    
    LOG_INF("Initializing RF Sensing occupancy sensor");

    // Initialize timers for deferred processing
    k_timer_init(&mModeIndicatorTimer, ModeIndicatorTimerHandler, nullptr);
    
    // Set initial mode and update LED
    mCurrentMode = RFS_MODE_STOPPED;
    UpdateModeIndicatorLED();

    // Initialize Matter OccupancySensing cluster instance with RF Sensing feature
    CHIP_ERROR err = InitializeClusterInstance(BitMask<OccupancySensing::Feature, uint32_t>(OccupancySensing::Feature::kRFSensing));
    if (err != CHIP_NO_ERROR) {
        return err;
    }

    // Initialize Channel Sounding
    int ret = channel_sounding_init(OccupancySensorRFS::RfsEventHandler);
    if (ret) {
        LOG_ERR("Channel Sounding initialization failed (err %d)", ret);
    }

    mInitialized = true;
    LOG_INF("RF Sensing occupancy sensor initialized successfully");

    return CHIP_NO_ERROR;
}

void OccupancySensorRFS::SetRFSMode(rfs_mode_t mode)
{
    if (mode == mCurrentMode) {
        LOG_DBG("Already in requested mode: %d", mode);
        return;
    }

    if(!ConnectivityMgrImpl().IsIPv6NetworkProvisioned() ||
                !ConnectivityMgrImpl().IsIPv6NetworkEnabled()) {
        LOG_INF("Thread not ready - only allowing STOPPED mode");
        if (mode != RFS_MODE_STOPPED) {
            return;
        }
    }

    rfs_mode_t old_mode = mCurrentMode;
    mCurrentMode = mode;
    
    const char* old_mode_str = (old_mode == RFS_MODE_NORMAL) ? "NORMAL" : 
                              (old_mode == RFS_MODE_LOW_POWER) ? "LOW_POWER" : "STOPPED";
    const char* new_mode_str = (mode == RFS_MODE_NORMAL) ? "NORMAL" : 
                              (mode == RFS_MODE_LOW_POWER) ? "LOW_POWER" : "STOPPED";
    
    LOG_INF("RFS Mode changed: %s -> %s", old_mode_str, new_mode_str);
    
    // Update LED indication
    UpdateModeIndicatorLED();
 
    // Get the sensor instance using singleton pattern
    OccupancySensorRFS *sensor = &OccupancySensorRFS::Instance();
    
    // Adjust RF sensing behavior based on mode
    switch (mode) {
        case RFS_MODE_NORMAL:
            Nrf::PostTask([sensor] {
                for (chip::EndpointId endpoint = kRfsFirstEndpoint; endpoint <= kRfsLastEndpoint; endpoint++) {
                    CHIP_ERROR err = SetHoldTime(endpoint, kRFSensingHoldTimeNormal);
                    if (err != CHIP_NO_ERROR) {
                        LOG_ERR("Failed to set RF Sensing occupancy state: %" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            });
            channel_sounding_set_inactive_interval(CONFIG_RFS_SENSING_NORMAL_INACTIVE_INTERVAL_MS);
            channel_sounding_procedure_enable(true);
            break;
        case RFS_MODE_LOW_POWER:
            Nrf::PostTask([sensor] {
                for (chip::EndpointId endpoint = kRfsFirstEndpoint; endpoint <= kRfsLastEndpoint; endpoint++) {
                    CHIP_ERROR err = SetHoldTime(endpoint, kRFSensingHoldTimeLowPower);
                    if (err != CHIP_NO_ERROR) {
                        LOG_ERR("Failed to set RF Sensing occupancy state: %" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            });
            channel_sounding_set_inactive_interval(CONFIG_RFS_SENSING_LOW_POWER_INACTIVE_INTERVAL_MS);
            channel_sounding_procedure_enable(true);
            break;
            
        case RFS_MODE_STOPPED:
            channel_sounding_procedure_enable(false);
            break;
    }
}

void OccupancySensorRFS::ToggleRFSMode()
{
    rfs_mode_t new_mode;
    
    switch (mCurrentMode) {
        case RFS_MODE_NORMAL:
            new_mode = RFS_MODE_LOW_POWER;
            break;
        case RFS_MODE_LOW_POWER:
            new_mode = RFS_MODE_STOPPED;
            break;
        case RFS_MODE_STOPPED:
        default:
            new_mode = RFS_MODE_NORMAL;
            break;
    }
    
    SetRFSMode(new_mode);
}

void OccupancySensorRFS::HandleButtonEvent(bool button_pressed)
{
    static bool button_handled = false;
    
    if (button_pressed && !button_handled) {
        // Button press - toggle mode
        LOG_DBG("Button 1 pressed - toggling RFS mode");
        ToggleRFSMode();
        button_handled = true;
    } else if (!button_pressed) {
        // Button released - reset handler flag
        button_handled = false;
    }
}

void OccupancySensorRFS::UpdateModeIndicatorLED()
{
    switch (mCurrentMode) {
        case RFS_MODE_NORMAL:
            // Stop blinking timer and set LED1 solid on
            k_timer_stop(&mModeIndicatorTimer);
            dk_set_led_on(DK_LED2);
            LOG_INF("LED1: Normal mode - solid ON");
            break;
            
        case RFS_MODE_LOW_POWER:
            // Start blinking timer - LED will blink every 1 second
            mLedBlinkState = false;
            dk_set_led_off(DK_LED2);
            k_timer_start(&mModeIndicatorTimer, K_MSEC(1000), K_MSEC(1000));
            LOG_INF("LED1: Low power mode - blinking every 1s");
            break;
            
        case RFS_MODE_STOPPED:
            // Stop blinking timer and turn LED1 off
            k_timer_stop(&mModeIndicatorTimer);
            dk_set_led_off(DK_LED2);
            LOG_INF("LED1: Stopped mode - OFF");
            break;
            
        default:
            LOG_ERR("Unknown RFS mode: %d", mCurrentMode);
            break;
    }
}

void OccupancySensorRFS::ModeIndicatorTimerHandler(k_timer *timer)
{
    ARG_UNUSED(timer);
    
    // Get the sensor instance (singleton)
    OccupancySensorRFS *sensor = &OccupancySensorRFS::Instance();
    
    if (sensor->mCurrentMode == RFS_MODE_LOW_POWER) {
        sensor->mLedBlinkState = !sensor->mLedBlinkState;
        if (sensor->mLedBlinkState) {
            dk_set_led_on(DK_LED2);
        } else {
            dk_set_led_off(DK_LED2);
        }
    }
}

