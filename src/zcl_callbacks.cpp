/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "board/board.h"

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
    EndpointId endpointId     = attributePath.mEndpointId;
    ClusterId clusterId     = attributePath.mClusterId;
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
		if (endpointId == OccupancySensorPIR::Instance().GetEndpointId()) {
			Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED3).Set(*value);
#if defined(CONFIG_RFS_ACTIVATED_BY_PIR)
			if (occupancy & (uint8_t)OccupancySensing::OccupancyBitmap::kOccupied) {
				// if PIR is occupied but RFS is not, start RFS sensing
				if (!OccupancySensorRFS::Instance().IsOccupied()) {
					OccupancySensorRFS::Instance().SetRFSMode(RFS_MODE_NORMAL);
				}
				// if PIR is occupied and RFS is occupied, do nothing
			} else {
				// if PIR is not occupied and RFS is not occupied, stop RFS sensing
				if (!OccupancySensorRFS::Instance().IsOccupied()) {
					OccupancySensorRFS::Instance().SetRFSMode(RFS_MODE_STOPPED);
				}
				// if PIR is not occupied but RFS is occupied, do nothing
			}
#endif // CONFIG_RFS_ACTIVATED_BY_PIR
		} else if (endpointId == OccupancySensorRFS::Instance().GetEndpointId()) {
			Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED4).Set(*value);
#if defined(CONFIG_RFS_ACTIVATED_BY_PIR)
			if (!(occupancy & (uint8_t)OccupancySensing::OccupancyBitmap::kOccupied)) {
				// if RFS is not occupied and PIR is not occupied, stop RFS sensing
				if (!OccupancySensorPIR::Instance().IsOccupied()) {
					OccupancySensorRFS::Instance().SetRFSMode(RFS_MODE_STOPPED);
				}
			}
#endif // CONFIG_RFS_ACTIVATED_BY_PIR
		}
	}
}

void emberAfOccupancySensingClusterInitCallback(EndpointId endpointId)
{
	if (endpointId > emberAfEndpointCount()) {
		ChipLogProgress(Zcl, "Invalid endpointId: %u exceeds maximum endpoint count", endpointId);
		return;
	}

	uint16_t holdTime = CONFIG_HOLD_TIME_LIMIT_DEFAULT_SEC;
	OccupancySensing::Structs::HoldTimeLimitsStruct::Type holdTimeLimits = {
		.holdTimeMin     = CONFIG_HOLD_TIME_LIMIT_MIN_SEC,
		.holdTimeMax     = CONFIG_HOLD_TIME_LIMIT_MAX_SEC,
		.holdTimeDefault = holdTime,
	};
	SetHoldTimeLimits(endpointId, holdTimeLimits);
	SetHoldTime(endpointId, holdTime);
}
