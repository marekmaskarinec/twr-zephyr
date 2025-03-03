#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static const struct gpio_dt_spec m_led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int led_set(int value)
{
	return gpio_pin_set_dt(&m_led0, value);
}

int led_blink(k_timeout_t delay)
{
	int ret;

	ret = led_set(1);
	if (ret) {
		LOG_ERR("Call `led_set` failed: %d", ret);
		return ret;
	}

	k_sleep(delay);

	ret = led_set(0);
	if (ret) {
		LOG_ERR("Call `led_set` failed: %d", ret);
		return ret;
	}

	return 0;
}

int led_init(void)
{
	int ret;

	ret = gpio_pin_configure_dt(&m_led0, GPIO_OUTPUT);
	if (ret) {
		LOG_ERR("Call `gpio_pin_configure_dt` failed: %d", ret);
		return ret;
	}

	return 0;
}