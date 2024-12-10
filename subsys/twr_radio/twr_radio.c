#include <twr/twr_radio.h>
#include "twr_radio_priv.h"
#include <drivers/spirit1.h>
#include <drivers/atsha204.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

/* Standard includes */
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(twr_radio, CONFIG_TWR_RADIO_LOG_LEVEL);

struct send_msg {
	void *queue_reserved;
	struct k_sem ack_event;
	uint16_t id;
	void *data;
	size_t data_len;
};

static K_QUEUE_DEFINE(m_send_queue);

static uint16_t m_msg_id_counter = 0;
static atomic_t m_busy;
static struct send_msg *m_current_msg;

static const struct device *m_spirit1 = DEVICE_DT_GET(DT_NODELABEL(spirit1));
static uint64_t m_radio_id;

static uint8_t *radio_id_to_buffer(uint8_t *buffer, uint64_t id)
{
	uint64_t id_be = sys_cpu_to_le48(id);
	memcpy(buffer, &id_be, 6);

	return buffer + 6;
}

static uint8_t *buffer_to_radio_id(uint8_t *buffer, uint64_t *id)
{
	uint64_t id_be = 0;
	memcpy(&id_be, buffer, 6);

	*id = sys_le48_to_cpu(id_be);

	return buffer + 6;
}

static uint8_t *msg_id_to_buffer(uint8_t *buffer, uint16_t id)
{
	uint16_t id_be = sys_cpu_to_le16(id);
	memcpy(buffer, &id_be, 2);

	return buffer + 2;
}

static uint8_t *buffer_to_msg_id(uint8_t *buffer, uint16_t *id)
{
	uint16_t id_be = 0;
	memcpy(&id_be, buffer, 2);

	*id = sys_le16_to_cpu(id_be);

	return buffer + 2;
}

static uint8_t *compose_header(uint8_t *buffer, uint16_t msg_id)
{
	buffer = radio_id_to_buffer(buffer, m_radio_id);
	buffer = msg_id_to_buffer(buffer, msg_id);
	return buffer;
}

static int send_msg(struct send_msg *msg)
{
	static uint8_t buf[96];

	uint8_t *ptr = compose_header(buf, msg->id);
	memcpy(ptr, msg->data, msg->data_len);
	size_t msg_size = msg->data_len + (ptr - buf);

	if (msg_size > sizeof(buf)) {
		return -ENOMEM;
	}

	m_current_msg = msg;

	LOG_HEXDUMP_INF(buf, msg_size, "Sending data:");

	int ret = spirit1_tx(m_spirit1, true, buf, msg_size);
	if (ret) {
		return ret;
	}

	return 0;
}

static int push_msg(struct send_msg *msg)
{
	if (!atomic_flag_test_and_set(&m_busy)) {
		return send_msg(msg);
	}

	k_queue_append(&m_send_queue, msg);

	return 0;
}

int twr_radio_send(void *data, size_t data_len, k_timeout_t timeout)
{
	int ret;
	struct send_msg msg = {
		.id = ++m_msg_id_counter,
		.data = data,
		.data_len = data_len,
	};

	ret = k_sem_init(&msg.ack_event, 0, 1);
	if (ret) {
		return ret;
	}

	ret = push_msg(&msg);
	if (ret) {
		return ret;
	}

	if (k_sem_take(&msg.ack_event, timeout)) {
		return -ETIMEDOUT;
	}

	atomic_clear(&m_busy);

	return 0;
}

static int receive_msg(uint8_t *buf, size_t len)
{
	uint64_t radio_id;
	uint8_t *ptr = buffer_to_radio_id(buf, &radio_id);
	if (radio_id != m_radio_id) {
		return 0;
	}

	uint16_t msg_id = 0;
	ptr = buffer_to_msg_id(ptr, &msg_id);

	size_t msg_len = len - (ptr - buf);

	LOG_HEXDUMP_INF(ptr, msg_len, "Received data:");

	if (ptr[0] == TWR_RADIO_HEADER_ACK) {
		if (m_current_msg && m_current_msg->id == msg_id) {
			k_sem_give(&m_current_msg->ack_event);
			m_current_msg = NULL;
		}
		return 0;
	}

	// TODO handle sub data and stuff
	return 0;
}

static int spirit_event_cb(enum spirit1_event event, void *user_data)
{
	int ret;

	switch (event) {
	case SPIRIT1_EVENT_RX_DATA_READY:
		LOG_INF("SPIRIT1_EVENT_RX_DATA_READY\n");
		static uint8_t buf[96];

		ret = spirit1_get_rx_data(m_spirit1, buf, sizeof(buf));
		if (ret < 0) {
			LOG_ERR("Call `spirit1_get_rx_data` failed: %d", ret);
			return ret;
		}

		return receive_msg(buf, ret);
	case SPIRIT1_EVENT_RX_DATA_DISC:
		LOG_INF("SPIRIT1_EVENT_RX_DATA_DISC\n");
		break;
	case SPIRIT1_EVENT_RX_TIMEOUT:
		LOG_INF("SPIRIT1_EVENT_RX_TIMEOUT\n");
		break;
	case SPIRIT1_EVENT_TX_DATA_SENT:
		LOG_INF("SPIRIT1_EVENT_TX_DATA_SENT\n");

		spirit1_rx(m_spirit1, K_SECONDS(1));

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

static int init(void)
{
	int ret;

	LOG_INF("System initialization");

	atomic_clear(&m_busy);

	const struct device *atsha204 =
		DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(microchip_atsha204));
	if (!device_is_ready(atsha204)) {
		LOG_ERR("ATSHA204 device not ready");
		return -ENODEV;
	}

	ret = atsha204_read_sn(atsha204, &m_radio_id);
	if (ret) {
		LOG_ERR("Call `atsha204_read_sn` failed: %d", ret);
	}
	m_radio_id = m_radio_id & 0xffffffffffff;

	if (!device_is_ready(m_spirit1)) {
		LOG_ERR("SPIRIT1 device not ready");
		return -ENODEV;
	}

	ret = spirit1_config(m_spirit1,
#if defined(CONFIG_TWR_RADIO_BAND_868)
			     SPIRIT1_BAND_868,
#elif defined(CONFIG_TWR_RADIO_BAND_915)
			     SPIRIT1_BAND_915,
#endif
			     0);
	if (ret) {
		LOG_ERR("Call `spirit1_config` failed: %d", ret);
	}

	spirit1_set_event_cb(m_spirit1, spirit_event_cb, NULL);

	return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);