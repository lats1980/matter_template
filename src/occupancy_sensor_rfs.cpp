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
#include <dk_buttons_and_leds.h>
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
                OccupancySensorRFS::Instance().SetRFSMode(RFS_MODE_NORMAL);
            } else {
                LOG_INF("Thread not ready - stopping Channel Sounding procedure");
                OccupancySensorRFS::Instance().SetRFSMode(RFS_MODE_STOPPED);
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

    // Reset device cache to force fresh detection
    ResetDeviceCache();
    
    LOG_INF("Initializing RF Sensing occupancy sensor");

    // Initialize work queue and timers for deferred processing
    k_work_init_delayable(&mRfsWork, RfsWorkHandler);
    k_timer_init(&mModeIndicatorTimer, ModeIndicatorTimerHandler, nullptr);
    
    // Set initial mode and update LED
    mCurrentMode = RFS_MODE_STOPPED;
    UpdateModeIndicatorLED();

    // Initialize Matter OccupancySensing cluster instance with RF Sensing feature
    CHIP_ERROR err = InitializeClusterInstance(BitMask<OccupancySensing::Feature, uint32_t>(OccupancySensing::Feature::kRFSensing));
    if (err != CHIP_NO_ERROR) {
        return err;
    }
    
    ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(MatterEventHandler, 0));

    // Initialize Channel Sounding
    int ret = channel_sounding_init();
    if (ret) {
        LOG_ERR("Channel Sounding initialization failed (err %d)", ret);
    }

    mInitialized = true;
    LOG_INF("RF Sensing occupancy sensor initialized successfully");

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

    if (channel_sounding_procedure_enable(true) != 0) {
        LOG_ERR("Failed to enable Channel Sounding procedures");
        return CHIP_ERROR_INTERNAL;
    }
    
    mRfSensingActive = true;

    return CHIP_NO_ERROR;
}

void OccupancySensorRFS::StopRFSensing()
{
    if (!mRfSensingActive) {
        return;
    }
    if (channel_sounding_procedure_enable(false) != 0) {
        LOG_ERR("Failed to disable Channel Sounding procedures");
        return;
    }
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
    
    if (sensor->mRfSensingActive) {
        // Check RF sensing data and determine occupancy
        bool occupied = sensor->CheckRFSensing();
        
        // Update occupancy state if occupied
        LOG_DBG("RFS occupied: %d", occupied);
        if (occupied) {
            Nrf::PostTask([sensor] { 
                CHIP_ERROR err = sensor->SetOccupancyState(true);
                if (err != CHIP_NO_ERROR) {
                    LOG_ERR("Failed to set RF Sensing occupancy state: %" CHIP_ERROR_FORMAT, err.Format());
                }
            });
        }
        sensor->StopRFSensing();
        // Reschedule next check based on mode
        if (sensor->mCurrentMode == RFS_MODE_NORMAL ) {
            k_work_schedule(&sensor->mRfsWork, K_MSEC(5000));
        } else if (sensor->mCurrentMode == RFS_MODE_LOW_POWER) {
            k_work_schedule(&sensor->mRfsWork, K_MSEC(25000));
        } else {
            // Stopped mode - do not reschedule
        }
    } else {
        sensor->StartRFSensing();
        k_work_schedule(&sensor->mRfsWork, K_MSEC(5000));
    }
}

chip::EndpointId OccupancySensorRFS::GetEndpointId() const
{
    uint8_t device = DetectConnectedDevice();
    
    switch (device) {
        case 1:
            return kDevice1EndpointId; // Endpoint 2
        case 2:
            return kDevice2EndpointId; // Endpoint 3
        default:
            LOG_WRN("Unknown device, return invalid endpoint");
            return kInvalidEndpointId;
    }
}

uint8_t OccupancySensorRFS::DetectConnectedDevice() const
{   
    // Get the remote connected device's MAC address from channel sounding module
    uint8_t remote_addr[6];
    bool addr_valid = channel_sounding_get_remote_address(remote_addr);
    
    if (addr_valid) {
        // Compare with known device MAC addresses
        if (memcmp(remote_addr, kDeviceMac1, 6) == 0) {
            LOG_INF("Detected Device 1 - using endpoint 2");
            mConnectedDevice = 1;
            return 1;
        } else if (memcmp(remote_addr, kDeviceMac2, 6) == 0) {
            LOG_INF("Detected Device 2 - using endpoint 3");
            mConnectedDevice = 2;
            return 2;
        }
        
        // Log the actual remote MAC address for debugging
        LOG_WRN("Detected unknown remote device MAC: %02X:%02X:%02X:%02X:%02X:%02X - filtering should have prevented this", 
                remote_addr[5], remote_addr[4], remote_addr[3],
                remote_addr[2], remote_addr[1], remote_addr[0]);
                
        // Default to device 1 to avoid invalid endpoint issues
        LOG_INF("Defaulting unknown device to Device 1 (endpoint 2)");
        mConnectedDevice = 1;
        return 1;
    } else {
        LOG_WRN("Failed to get remote device MAC address from channel sounding");
    }
    
    // If we can't get the address, default to device 1
    LOG_WRN("Device detection failed, defaulting to Device 1 (endpoint 2)");
    mConnectedDevice = 1;
    return 1;
}

void OccupancySensorRFS::SetRFSMode(rfs_mode_t mode)
{
    if (mode == mCurrentMode) {
        LOG_DBG("Already in requested mode: %d", mode);
        return;
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
    
    // Adjust RF sensing behavior based on mode
    switch (mode) {
        case RFS_MODE_NORMAL:
        case RFS_MODE_LOW_POWER:
            if (!mRfSensingActive) {
                k_work_reschedule(&mRfsWork, K_NO_WAIT);
            }
            break;
            
        case RFS_MODE_STOPPED:
            k_work_cancel_delayable(&mRfsWork);
            StopRFSensing();
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

