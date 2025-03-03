#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <twr/twr_dice.h>
#include <twr/twr_radio.h>

#include "accel.h"

LOG_MODULE_REGISTER(accel, LOG_LEVEL_INF);

static const struct device *m_dev = DEVICE_DT_GET_ANY(st_lis2dh);
static struct twr_dice m_dice;

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	int ret;

	ret = sensor_sample_fetch(m_dev);
	if (ret) {
		LOG_ERR("Call `sensor_sample_fetch` failed: %d", ret);
		return;
	}

	struct sensor_value accel[3];
	ret = sensor_channel_get(m_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret) {
		LOG_ERR("Call `sensor_channel_get` failed: %d", ret);
		return;
	}

	double accel_x = sensor_value_to_double(&accel[0]);
	double accel_y = sensor_value_to_double(&accel[1]);
	double accel_z = sensor_value_to_double(&accel[2]);

	enum twr_dice_face last_face = twr_dice_get_face(&m_dice);
	twr_dice_feed_vectors(&m_dice, accel_x, accel_y, accel_z);
	enum twr_dice_face face = twr_dice_get_face(&m_dice);
	if (last_face == face) {
		return;
	}

	LOG_INF("Orientation changed to %d", face);

	static struct twr_radio_msg msg;
	memset(&msg, 0, sizeof(struct twr_radio_msg));
	msg.type = TWR_RADIO_MSG_TYPE_PUB;
	msg.header = TWR_RADIO_HEADER_PUB_TOPIC_INT;
	int32_t face_le = sys_cpu_to_le32(face);
	memcpy(msg.pub.data, &face_le, sizeof(face_le));
	static const char topic[] = "orientation";
	memcpy(msg.pub.data + sizeof(int32_t), topic, sizeof(topic));
	msg.pub.data_len = sizeof(int32_t) + sizeof(topic);

	ret = twr_radio_pub(&msg, K_MSEC(1000));
	if (ret) {
		LOG_ERR("Call `twr_radio_pub` failed: %d", ret);
	}
}

int accel_init(void)
{
	int ret;

	if (!device_is_ready(m_dev)) {
		return -ENODEV;
	}

	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	ret = sensor_trigger_set(m_dev, &trig, trigger_handler);
	if (ret) {
		LOG_ERR("Call `sensor_trigger_set` failed: %d", ret);
		return ret;
	}

	twr_dice_init(&m_dice, TWR_DICE_FACE_UNKNOWN);

	return 0;
}