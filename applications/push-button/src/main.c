#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <twr/twr_radio.h>

#include "button.h"
#include "led.h"
#include "therm.h"
#include "accel.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int send_pairing(void)
{
	struct twr_radio_msg msg = {
		.header = TWR_RADIO_HEADER_PAIRING,
		.type = TWR_RADIO_MSG_TYPE_PUB,
	};

	memset(msg.pub.data, 0, sizeof(msg.pub.data));
	msg.pub.data[0] = strlen(CONFIG_FW_NAME);
	strcpy(&msg.pub.data[1], CONFIG_FW_NAME);
	strcpy(&msg.pub.data[1 + strlen(CONFIG_FW_NAME) + 1], CONFIG_FW_VERSION);
	msg.pub.data_len = 1 + strlen(CONFIG_FW_NAME) + 1 + strlen(CONFIG_FW_VERSION) + 1;

	int ret = twr_radio_pub(&msg, K_MSEC(1000));
	if (ret) {
		LOG_ERR("Call `twr_radio_pub` failed: %d", ret);
		return ret;
	}

	return 0;
}

int main(void)
{
	int ret;

	ret = led_init();
	if (ret) {
		LOG_ERR("Call `led_init` failed: %d", ret);
		return ret;
	}

	led_set(1);

	ret = send_pairing();
	if (ret) {
		LOG_ERR("Call `send_pairing` failed: %d", ret);
		return ret;
	}

	ret = therm_init();
	if (ret) {
		LOG_ERR("Call `therm_init` failed: %d", ret);
		return ret;
	}

	ret = accel_init();
	if (ret) {
		LOG_ERR("Call `accel_init` failed: %d", ret);
		return ret;
	}

	ret = button_init();
	if (ret) {
		LOG_ERR("Call `button_init` failed: %d", ret);
		return ret;
	}

	led_set(0);

	while (true) {
		LOG_INF("Alive!");
		k_sleep(K_SECONDS(5));
	}
}
