/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "lib/core/CHIPError.h"
#include "lib/support/CodeUtils.h"
#include "channel_sounding_ras_initiator.h"

#include <setup_payload/OnboardingCodesUtil.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

#define MODE_BUTTON_MASK DK_BTN2_MSK

void AppTask::ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged)
{
	if (MODE_BUTTON_MASK & hasChanged) {
		bool button_pressed = (MODE_BUTTON_MASK & state) != 0;
		Nrf::PostTask([button_pressed] { 
			OccupancySensorRFS::Instance().HandleButtonEvent(button_pressed);
		});
	}
}

void AppTask::MatterEventHandler(const ChipDeviceEvent *event, intptr_t /* unused */)
{
	switch (event->Type) {
		case DeviceEventType::kThreadStateChange:
			if(ConnectivityMgrImpl().IsIPv6NetworkProvisioned() &&
						ConnectivityMgrImpl().IsIPv6NetworkEnabled()) {
				LOG_INF("Thread is provisioned and enabled");
#if !defined(CONFIG_RFS_ACTIVATED_BY_PIR)
				LOG_INF("Enabling Channel Sounding procedure");
				channel_sounding_procedure_enable(true);
#endif
			} else {
				LOG_INF("Thread is not ready");
#if !defined(CONFIG_RFS_ACTIVATED_BY_PIR)
				LOG_INF("Disabling Channel Sounding procedure");
				channel_sounding_procedure_enable(false);
#endif	
			}
			break;
	default:
		break;
	}
}

CHIP_ERROR AppTask::Init()
{
	/* Initialize Matter stack */
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(MatterEventHandler, 0));

	/* Initialize RF Sensing occupancy sensor */
	ReturnErrorOnFailure(OccupancySensorRFS::Instance().Init());

#if defined(CONFIG_PIR_SUPPORT)
	/* Initialize PIR occupancy sensor */
	ReturnErrorOnFailure(OccupancySensorPIR::Instance().Init());
#endif

	return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
