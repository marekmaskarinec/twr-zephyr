#include <drivers/atsha204.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>

#include <stdint.h>

#define DT_DRV_COMPAT microchip_atsha204

LOG_MODULE_REGISTER(ATSHA204, CONFIG_ATSHA204_LOG_LEVEL);

struct data {
	struct k_mutex lock;
};

struct config {
	const struct i2c_dt_spec bus;
};

// This CRC function is taken from the original twr-atsha204 driver, since I couldn't reproduce it
// using Zephyr CRC functions.
static uint16_t crc16(uint8_t *buffer, uint8_t length)
{
	uint16_t crc16;
	uint8_t shift_register;
	uint8_t data;
	uint8_t data_bit;
	uint8_t crc_bit;

	for (crc16 = 0; length != 0; length--, buffer++) {
		data = *buffer;

		for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {

			data_bit = (data & shift_register) ? 1 : 0;

			crc_bit = crc16 >> 15;

			crc16 <<= 1;

			if (data_bit != crc_bit) {
				crc16 ^= 0x8005;
			}
		}
	}

	return crc16;
}

static int wakeup(const struct device *dev)
{
	const struct config *cfg = dev->config;
	int ret;

	LOG_DBG("Wake up pulse");

	uint32_t config_orig;
	ret = i2c_get_config(cfg->bus.bus, &config_orig);
	if (ret) {
		LOG_ERR("Call `i2c_get_config` failed: %d", ret);
		return ret;
	}

	uint32_t config = (config_orig & ~I2C_SPEED_MASK) | I2C_SPEED_SET(I2C_SPEED_STANDARD);
	ret = i2c_configure(cfg->bus.bus, config);
	if (ret) {
		LOG_ERR("Call `i2c_configure` failed: %d", ret);
		return ret;
	}

	ret = i2c_read(cfg->bus.bus, NULL, 0, 0x00);
	if (ret) {
		i2c_read(cfg->bus.bus, NULL, 0, 0x00);
	}

	ret = i2c_configure(cfg->bus.bus, config_orig);
	if (ret) {
		LOG_ERR("Call `i2c_configure` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int send_command(const struct device *dev, uint8_t op, uint8_t par0, uint16_t par1)
{
	const struct config *cfg = dev->config;

	uint8_t cmd[8];
	cmd[0] = 0x03;
	cmd[1] = sizeof(cmd) - 1;
	cmd[2] = op;
	cmd[3] = par0;
	cmd[4] = par1 & 0xff;
	cmd[5] = par1 >> 8;

	uint16_t crc = crc16(cmd + 1, sizeof(cmd) - 3);

	cmd[6] = crc & 0xff;
	cmd[7] = crc >> 8;

	LOG_HEXDUMP_DBG(cmd, sizeof(cmd), "> ");

	int ret = i2c_write_dt(&cfg->bus, cmd, sizeof(cmd));
	if (ret) {
		wakeup(dev);

		ret = i2c_write_dt(&cfg->bus, cmd, sizeof(cmd));
	}

	return ret;
}

static int read(const struct device *dev, void *data, size_t len)
{
	const struct config *cfg = dev->config;

	uint8_t buf[32];
	// 1 byte for length, 2 bytes for CRC
	size_t msglen = len + 3;

	if (msglen >= sizeof(buf)) {
		return -ENOMEM;
	}

	int ret = i2c_read_dt(&cfg->bus, buf, msglen);
	if (ret) {
		LOG_ERR("Call `i2c_read_dt` failed: %d", ret);
	}

	LOG_HEXDUMP_DBG(buf, msglen, "< ");

	if (buf[0] != msglen) {
		return -EBADMSG;
	}

	uint16_t crc_check = crc16(buf, len + 1);
	uint16_t crc_msg = (buf[len + 2] << 8) | buf[len + 1];
	if (crc_check != crc_msg) {
		return -EBADMSG;
	}

	memcpy(data, &buf[1], len);
	return 0;
}

static int read_sn_(const struct device *dev, uint64_t *sn)
{
	int ret;
	struct data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t snbuf[8];

	ret = send_command(dev, 0x02, 0, 0x00);
	if (ret) {
		LOG_ERR("Call `send_command` failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}
	k_msleep(4);

	ret = read(dev, snbuf, 4);
	if (ret) {
		LOG_ERR("Call `read` failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = send_command(dev, 0x02, 0, 0x02);
	if (ret) {
		LOG_ERR("Call `send_command` failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}
	k_msleep(4);

	ret = read(dev, snbuf + 4, 4);
	if (ret) {
		LOG_ERR("Call `read` failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	k_mutex_unlock(&data->lock);

	memcpy(sn, snbuf, sizeof(snbuf));
	*sn = sys_be64_to_cpu(*sn);

	return 0;
}

static int init(const struct device *dev)
{
	const struct config *cfg = dev->config;
	struct data *data = dev->data;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
		return -EINVAL;
	}

	k_mutex_init(&data->lock);

	return 0;
}

static const struct atsha204_driver_api atsha204_driver_api = {
	.read_sn = read_sn_,
};

#define ATSHA204_INST(inst)                                                                        \
	static struct data atsha204_data_##inst;                                                   \
	static const struct config atsha204_config_##inst = {                                      \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, init, NULL, &atsha204_data_##inst, &atsha204_config_##inst,    \
			      POST_KERNEL, CONFIG_ATSHA204_INIT_PRIORITY, &atsha204_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ATSHA204_INST)