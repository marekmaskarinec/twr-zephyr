/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <twr/twr_radio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	uint8_t buf[64];
	buf[0] = TWR_RADIO_HEADER_PAIRING;

	const char *fw_name = "Zephyr test";
	buf[1] = strlen(fw_name);
	strcpy((char *)&buf[2], fw_name);
	strcpy((char *)&buf[2 + strlen(fw_name) + 1], "v1.0.0");

	int ret = twr_radio_send(buf, 2 + strlen(fw_name) + 1 + strlen("v1.0.0"), K_MSEC(1000));
	if (ret) {
		LOG_ERR("twr_radio_send failed: %d", ret);
	} else {
		LOG_INF("twr_radio_send OK");
	}
}
