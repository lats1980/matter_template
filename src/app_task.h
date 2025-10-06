/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <platform/CHIPDeviceLayer.h>
#include "occupancy_sensor_pir.h"
#include "occupancy_sensor_rfs.h"

#include "board/led_widget.h"
#include "board/board.h"

struct k_timer;

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

private:
	CHIP_ERROR Init();
	static void ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged);
	static void MatterEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t data);
};
