/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <cstdint>
#include <zephyr/kernel.h>

class RelayWidget {
public:
	int Init();
	void Set(bool state);
	void Invert();

private:
	uint32_t mGPIONum;
	bool mState;
	void DoSet(bool state);
};