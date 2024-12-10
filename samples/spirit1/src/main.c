/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/spirit1.h>
#include <zephyr/logging/log.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *spirit1 = DEVICE_DT_GET(DT_NODELABEL(spirit1));

static int event_cb(enum spirit1_event event, void *user_data)
{
	switch (event) {
	case SPIRIT1_EVENT_RX_DATA_READY:
		LOG_INF("SPIRIT1_EVENT_RX_DATA_READY\n");
		uint8_t buf[64];
		int ret = spirit1_get_rx_data(spirit1, buf, sizeof(buf));
		if (ret < 0) {
			LOG_INF("Error %d\n", ret);
		} else {
			LOG_HEXDUMP_INF(buf, ret, "Received data:");
		}

		spirit1_rx(spirit1, K_MSEC(1000));

		break;
	case SPIRIT1_EVENT_RX_DATA_DISC:
		LOG_INF("SPIRIT1_EVENT_RX_DATA_DISC\n");
		break;
	case SPIRIT1_EVENT_RX_TIMEOUT:
		LOG_INF("SPIRIT1_EVENT_RX_TIMEOUT\n");
		spirit1_rx(spirit1, K_MSEC(1000));
		break;
	case SPIRIT1_EVENT_TX_DATA_SENT:
		LOG_INF("SPIRIT1_EVENT_TX_DATA_SENT\n");
		break;
	case SPIRIT1_EVENT_MAX_BO_CCA_REACH:
		LOG_INF("SPIRIT1_EVENT_MAX_BO_CCA_REACH\n");
		break;
	default:
		LOG_INF("Unknown event\n");
		break;
	}

	return 0;
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	ret = spirit1_config(spirit1, SPIRIT1_BAND_868, 0);
	if (ret < 0) {
		printf("Error %d\n", ret);
		return 0;
	}

	spirit1_set_event_cb(spirit1, event_cb, NULL);

	spirit1_rx(spirit1, K_MSEC(1000));

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
