/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "relay_widget.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

static const struct device *relay_dev;

#define RELAY_PORT gpio0
#define RELAY_GPIO 1

int RelayWidget::Init(void)
{
    int err;

	mGPIONum = RELAY_GPIO;
	relay_dev = DEVICE_DT_GET(DT_NODELABEL(RELAY_PORT));
	if (!device_is_ready(relay_dev)) {
		printk("GPIO controller not ready");
		return -EINVAL;
	}
	err = gpio_pin_configure(relay_dev, mGPIONum, GPIO_OUTPUT); 
	if (err != 0) {
		printk("Failed to configure pin %d\n", mGPIONum);
		return -EINVAL;
	}

	Set(false);
    return 0;
}

void RelayWidget::Invert()
{
	Set(!mState);
}

void RelayWidget::Set(bool state)
{
	DoSet(state);
}

void RelayWidget::DoSet(bool state)
{
	mState = state;
	gpio_pin_set(relay_dev, mGPIONum, mState);
}