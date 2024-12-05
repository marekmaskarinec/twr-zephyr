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

	uint8_t buf[64];

	ret = spirit1_config(spirit1, SPIRIT1_BAND_868, 0);
	if (ret < 0) {
		printf("Error %d\n", ret);
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		ret = spirit1_rx(spirit1, buf, sizeof(buf), K_FOREVER);
		if (ret < 0) {
			printf("Error %d\n", ret);
		} else {
			printf("Received %d bytes\n", ret);
			for (int i = 0; i < ret; i++) {
				printf("%02x ", buf[i]);
			}
			printf("\n");
			printf("%*s\n", ret, buf);
		}

		// k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
