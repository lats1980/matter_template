/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "board/board.h"
#include "channel_sounding_ras_initiator.h"
#include "app/task_executor.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/ConcreteAttributePath.h>

#include <lib/support/logging/CHIPLogging.h>

#include <app/clusters/occupancy-sensor-server/occupancy-hal.h>
#include <app/clusters/occupancy-sensor-server/occupancy-sensor-server.h>

#include <app/util/attribute-storage.h>
#include <app/util/endpoint-config-api.h>

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::app::Clusters;
using namespace ::chip::app::Clusters::OccupancySensing;

void MatterPostAttributeChangeCallback(const chip::app::ConcreteAttributePath &attributePath, uint8_t type,
					   uint16_t size, uint8_t *value)
{
	EndpointId endpointId	 = attributePath.mEndpointId;
	ClusterId clusterId	 = attributePath.mClusterId;
	AttributeId attributeId = attributePath.mAttributeId;
	ChipLogProgress(Zcl, "MatterPostAttributeChangeCallback - Cluster ID: " ChipLogFormatMEI
					", EndPoint ID: '0x%02x', Attribute ID: " ChipLogFormatMEI,
					ChipLogValueMEI(clusterId), endpointId, ChipLogValueMEI(attributeId));

	if (endpointId > emberAfEndpointCount()) {
		ChipLogProgress(Zcl, "Invalid endpointId: %u exceeds maximum endpoint count", endpointId);
		return;
	}

	if (clusterId == OccupancySensing::Id && attributeId == OccupancySensing::Attributes::Occupancy::Id) {
		uint8_t occupancy = *value;
		ChipLogProgress(Zcl, "Cluster OccupancySensing: attribute Occupancy set to %" PRIu8 "", occupancy);

		if (occupancy & (uint8_t)OccupancySensing::OccupancyBitmap::kOccupied) {
			ChipLogProgress(Zcl, "Occupancy State: OCCUPIED");
		} else {
			ChipLogProgress(Zcl, "Occupancy State: UNOCCUPIED");
		}

		if (OccupancySensorRFS::Instance().IsValidEndpoint(endpointId)) {
			// Update LED4 based on RF Sensing occupancy state. Turn on if any RFS endpoint is occupied
			Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED4).Set(OccupancySensorRFS::Instance().IsOccupied());
		}
#if defined(CONFIG_PIR_SUPPORT)
		else if (OccupancySensorPIR::Instance().IsValidEndpoint(endpointId)) {
			// Update LED3 based on PIR occupancy state
			Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED3).Set(OccupancySensorPIR::Instance().IsOccupied(endpointId));
#if defined(CONFIG_RFS_ACTIVATED_BY_PIR)
			if (occupancy & (uint8_t)OccupancySensing::OccupancyBitmap::kOccupied) {
				channel_sounding_procedure_enable(true);
				// PIR detected occupancy - set normal mode
				channel_sounding_set_inactive_interval(CONFIG_RFS_SENSING_NORMAL_INACTIVE_INTERVAL_MS);
				Nrf::PostTask([endpointId] {
					OccupancySensorRFS::Instance().SetHoldTime(CONFIG_HOLD_TIME_LIMIT_RFS_NORMAL_SEC);
				});
			} else {
				if (!OccupancySensorRFS::Instance().IsOccupied()) {
					// No occupancy from RFS - disable channel sounding
					channel_sounding_procedure_enable(false);
				} else {
					// Still occupied by RFS - switch to low power mode
					channel_sounding_set_inactive_interval(CONFIG_RFS_SENSING_LOW_POWER_INACTIVE_INTERVAL_MS);
					Nrf::PostTask([endpointId] {
						OccupancySensorRFS::Instance().SetHoldTime(CONFIG_HOLD_TIME_LIMIT_RFS_LOW_POWER_SEC);
					});
				}
			}
#endif // CONFIG_RFS_ACTIVATED_BY_PIR
		}
#endif // CONFIG_PIR_SUPPORT	
	}
}

void emberAfOccupancySensingClusterInitCallback(EndpointId endpointId)
{
	if (endpointId > emberAfEndpointCount()) {
		ChipLogProgress(Zcl, "Invalid endpointId: %u exceeds maximum endpoint count", endpointId);
		return;
	}

	uint16_t holdTime = CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;

	if (OccupancySensorRFS::Instance().IsValidEndpoint(endpointId)) {
		holdTime = CONFIG_HOLD_TIME_LIMIT_RFS_NORMAL_SEC;;
	}
	OccupancySensing::Structs::HoldTimeLimitsStruct::Type holdTimeLimits = {
		.holdTimeMin	 = CONFIG_HOLD_TIME_LIMIT_MIN_SEC,
		.holdTimeMax	 = CONFIG_HOLD_TIME_LIMIT_MAX_SEC,
		.holdTimeDefault = holdTime,
	};
	SetHoldTimeLimits(endpointId, holdTimeLimits);
	SetHoldTime(endpointId, holdTime);
}
