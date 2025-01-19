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
	struct twr_radio_msg msg = {
		.header = TWR_RADIO_HEADER_PAIRING,
		.type = TWR_RADIO_MSG_TYPE_PUB,
	};

	memset(msg.pub.data, 0, sizeof(msg.pub.data));
	msg.pub.data[0] = strlen("Zephyr test");
	strcpy((char *)&msg.pub.data[1], "Zephyr test");
	strcpy((char *)&msg.pub.data[1 + strlen("Zephyr test") + 1], "v1.0.0");
	// msg.pub.data[1 + strlen("Zephyr test") + 1 + strlen("v1.0.0") + 1] = 0x00;
	msg.pub.data_len = 1 + strlen("Zephyr test") + 1 + strlen("v1.0.0");

	int ret = twr_radio_pub(&msg, K_MSEC(1000));
	if (ret) {
		LOG_ERR("twr_radio_send failed: %d", ret);
	} else {
		LOG_INF("twr_radio_send OK");
	}

	int count = 0;
	while (true) {
		memset(&msg, 0, sizeof(msg));
		msg.type = TWR_RADIO_MSG_TYPE_PUB;
		msg.header = TWR_RADIO_HEADER_PUB_PUSH_BUTTON;
		msg.pub.data[0] = ++count;
		msg.pub.data_len = 1;

		ret = twr_radio_pub(&msg, K_MSEC(1000));
		if (ret) {
			LOG_ERR("twr_radio_send failed: %d", ret);
		} else {
			LOG_INF("twr_radio_send OK");
		}

		ret = twr_radio_recv(&msg, K_MSEC(3200));
		if (ret) {
			LOG_ERR("twr_radio_recv failed: %d", ret);
		} else {
			LOG_INF("twr_radio_recv OK");
			twr_radio_dump_msg(&msg);
		}
	}
}
