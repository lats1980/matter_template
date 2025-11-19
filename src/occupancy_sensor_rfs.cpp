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
    
    // Indicate initial mode via LED
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

void OccupancySensorRFS::HandleButtonEvent(bool button_pressed)
{
    static bool button_handled = false;
    
    if (button_pressed && !button_handled) {
        // Button press - toggle mode
        LOG_INF("Button 1 pressed - toggling RFS mode");
        if (channel_sounding_is_preemptive_mode() == true) {
            channel_sounding_set_preemptive_mode(false);
        } else {
            channel_sounding_set_preemptive_mode(true);
        }
        button_handled = true;
        UpdateModeIndicatorLED();
    } else if (!button_pressed) {
        // Button released - reset handler flag
        button_handled = false;
    }
}

void OccupancySensorRFS::UpdateModeIndicatorLED()
{
    if (channel_sounding_is_preemptive_mode()) {
        // Preemptive mode - start blinking timer - LED will blink every 1 second
        dk_set_led_off(DK_LED2);
        k_timer_start(&mModeIndicatorTimer, K_MSEC(1000), K_MSEC(1000));
        LOG_INF("LED2: Preemptive mode - blinking every 1s");
    } else {
        // Non-preemptive mode - stop blinking timer and set LED2 solid on
        k_timer_stop(&mModeIndicatorTimer);
        dk_set_led_on(DK_LED2);
        LOG_INF("LED2: Non Preemptive mode - solid ON");
    }
}

void OccupancySensorRFS::ModeIndicatorTimerHandler(k_timer *timer)
{
    ARG_UNUSED(timer);
    
    // Get the sensor instance (singleton)
    OccupancySensorRFS *sensor = &OccupancySensorRFS::Instance();
    
    sensor->mLedBlinkState = !sensor->mLedBlinkState;
    if (sensor->mLedBlinkState) {
        dk_set_led_on(DK_LED2);
    } else {
        dk_set_led_off(DK_LED2);
    }
}

bool OccupancySensorRFS::IsOccupied(chip::EndpointId endpointId) const
{
    if (endpointId != kInvalidEndpointId) {
        // Check specific endpoint
        if (!IsValidEndpoint(endpointId)) {
            LOG_ERR("Invalid endpoint ID: %u", endpointId);
            return false;
        }
        chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
        Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(endpointId, &currentOccupancy);
        VerifyOrDie(status == Protocols::InteractionModel::Status::Success);
        return currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied);
    } else {
        // Check all RF Sensing endpoints
        for (chip::EndpointId ep = kRfsFirstEndpoint; ep <= kRfsLastEndpoint; ++ep) {
            chip::BitMask<Clusters::OccupancySensing::OccupancyBitmap> currentOccupancy;
            Protocols::InteractionModel::Status status = OccupancySensing::Attributes::Occupancy::Get(ep, &currentOccupancy);
            VerifyOrDie(status == Protocols::InteractionModel::Status::Success);
            if (currentOccupancy.Has(Clusters::OccupancySensing::OccupancyBitmap::kOccupied)) {
                return true;
            }
        }
        return false;
    }
}

CHIP_ERROR OccupancySensorRFS::SetHoldTime(uint16_t holdTimeSec)
{
    // Set hold time for all RF Sensing endpoints
    for (chip::EndpointId ep = kRfsFirstEndpoint; ep <= kRfsLastEndpoint; ++ep) {
        CHIP_ERROR err = OccupancySensing::SetHoldTime(ep, holdTimeSec);
        if (err != CHIP_NO_ERROR) {
            LOG_ERR("Failed to set OccupancySensorType for endpoint %u: %" CHIP_ERROR_FORMAT, ep, err.Format());
            return err;
        }
    }
    LOG_INF("Set RFSHoldTime to %u seconds for all RF Sensing endpoints", holdTimeSec);

    return CHIP_NO_ERROR;
}
