#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <twr/twr_radio.h>

#include "therm.h"

LOG_MODULE_REGISTER(therm, LOG_LEVEL_INF);

const struct device *const m_tmp112 = DEVICE_DT_GET_ANY(ti_tmp112);

static int send(struct sensor_value *val)
{
	static struct twr_radio_msg msg;
	memset(&msg, 0, sizeof(struct twr_radio_msg));
	msg.type = TWR_RADIO_MSG_TYPE_PUB;
	msg.header = TWR_RADIO_HEADER_PUB_TEMPERATURE;
	msg.pub.data[0] = 0; // channel 0
	float temp = sensor_value_to_double(val);
	memcpy(msg.pub.data + 1, &temp, sizeof(temp));
	msg.pub.data_len = 1 + sizeof(temp);

	return twr_radio_pub(&msg, K_MSEC(1000));
}

static int sample(struct sensor_value *val)
{
	int ret;

	ret = sensor_sample_fetch(m_tmp112);
	if (ret) {
		printk("sensor_sample_fetch failed ret %d\n", ret);
		return ret;
	}

	ret = sensor_channel_get(m_tmp112, SENSOR_CHAN_AMBIENT_TEMP, val);
	if (ret) {
		printk("sensor_channel_get failed ret %d\n", ret);
		return ret;
	}

	return 0;
}

static void therm_work_handler(struct k_work *work)
{
	int ret;
	struct sensor_value val;

	ret = sample(&val);
	if (ret) {
		LOG_INF("sample failed ret %d\n", ret);
		return;
	}

	LOG_INF("Temperature: %d.%06d C\n", val.val1, val.val2);

	ret = send(&val);
	if (ret) {
		LOG_INF("send failed ret %d\n", ret);
		return;
	}
}

static K_WORK_DEFINE(m_therm_work, therm_work_handler);

static void therm_timer_handler(struct k_timer *timer)
{
	k_work_submit(&m_therm_work);
}

static K_TIMER_DEFINE(m_therm_timer, therm_timer_handler, NULL);

int therm_init(void)
{
	int ret;
	struct sensor_value attr;

	attr.val1 = 150;
	attr.val2 = 0;
	ret = sensor_attr_set(m_tmp112, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_FULL_SCALE, &attr);
	if (ret) {
		printk("sensor_attr_set failed ret %d\n", ret);
		return ret;
	}

	attr.val1 = 8;
	attr.val2 = 0;
	ret = sensor_attr_set(m_tmp112, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_SAMPLING_FREQUENCY,
			      &attr);
	if (ret) {
		printk("sensor_attr_set failed ret %d\n", ret);
		return ret;
	}

	k_timer_start(&m_therm_timer, K_SECONDS(5), K_MINUTES(30));

	return 0;
}