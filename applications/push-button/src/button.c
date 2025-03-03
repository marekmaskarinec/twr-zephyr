#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <twr/twr_radio.h>

#include <stdint.h>

#include "led.h"

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

#define DEBOUNCE_TIMEOUT_MS 20

static const struct gpio_dt_spec m_sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback m_sw0_cb;

static uint16_t m_count = 0;

void button_work_handler(struct k_work *work)
{
	m_count++;
	LOG_INF("Button clicked %d", m_count);
	led_blink(K_MSEC(30));

	static struct twr_radio_msg msg;
	memset(&msg, 0, sizeof(struct twr_radio_msg));
	msg.type = TWR_RADIO_MSG_TYPE_PUB;
	msg.header = TWR_RADIO_HEADER_PUB_EVENT_COUNT;
	msg.pub.data[0] = TWR_RADIO_PUB_EVENT_PUSH_BUTTON;
	uint16_t count_le = sys_cpu_to_le16(m_count);
	memcpy(msg.pub.data + 1, &count_le, sizeof(count_le));
	msg.pub.data_len = 1 + sizeof(count_le);

	int ret = twr_radio_pub(&msg, K_MSEC(1000));
	if (ret) {
		LOG_ERR("Call `twr_radio_pub` failed: %d", ret);
	}
}

static K_WORK_DEFINE(m_button_work, button_work_handler);

void debounce_timer_handler(struct k_timer *timer)
{
	k_work_submit(&m_button_work);
}

static K_TIMER_DEFINE(m_debounce_timer, debounce_timer_handler, NULL);

void sw0_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (!gpio_pin_get_dt(&m_sw0)) {
		k_timer_start(&m_debounce_timer, K_MSEC(DEBOUNCE_TIMEOUT_MS), K_FOREVER);
	} else {
		k_timer_stop(&m_debounce_timer);
	}
}

int button_init(void)
{
	int ret;

	ret = gpio_pin_configure_dt(&m_sw0, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Call `gpio_pin_configure_dt` failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&m_sw0,
					      GPIO_INT_EDGE_TO_ACTIVE | GPIO_INT_EDGE_TO_INACTIVE);
	if (ret) {
		LOG_ERR("Call `gpio_pin_interrupt_configure_dt` failed: %d", ret);
		return ret;
	}

	gpio_init_callback(&m_sw0_cb, sw0_cb_handler, BIT(m_sw0.pin));
	ret = gpio_add_callback_dt(&m_sw0, &m_sw0_cb);
	if (ret) {
		LOG_ERR("Call `gpio_add_callback_dt` failed: %d", ret);
		return ret;
	}

	return 0;
}