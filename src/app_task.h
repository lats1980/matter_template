/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "board/board.h"
#include "relay_widget.h"

#include <platform/CHIPDeviceLayer.h>

struct k_timer;

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

	void UpdateClusterState();
	RelayWidget GetRelay() { return mRelay; }

private:
	RelayWidget mRelay;
	CHIP_ERROR Init();
	static void ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged);
};
