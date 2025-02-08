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

static K_MUTEX_DEFINE(m_mutex);

static uint16_t m_msg_id_counter = 0;
static struct twr_radio_msg *m_recv_msg = NULL;

static K_SEM_DEFINE(m_tx_done_event, 1, 1);
static K_SEM_DEFINE(m_recv_event, 1, 1);

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

void twr_radio_dump_msg(struct twr_radio_msg *msg)
{
	LOG_INF("Message:");
	LOG_INF("  Header: %d", msg->header);
	LOG_INF("  Type: %d", msg->type);
	LOG_INF("  ID: %d", msg->id);

	switch (msg->type) {
	case TWR_RADIO_MSG_TYPE_PUB:
		LOG_HEXDUMP_INF(msg->pub.data, msg->pub.data_len, "  Data:");
		break;
	case TWR_RADIO_MSG_TYPE_NODE:
		LOG_HEXDUMP_INF(msg->node.data, msg->node.data_len, "  Data:");
		break;
	default:
		break;
	}
}

static int compose_pub_msg(struct twr_radio_msg *msg, uint8_t *buf, int len)
{
	if (len < 6 + 2 + 1 + msg->pub.data_len) {
		return -ENOMEM;
	}

	uint8_t *ptr = buf;

	ptr = radio_id_to_buffer(ptr, m_radio_id);
	ptr = msg_id_to_buffer(ptr, msg->id);
	*(ptr++) = msg->header;
	memcpy(ptr, msg->pub.data, msg->pub.data_len);

	return ptr - buf + msg->pub.data_len;
}

static int compose_msg(struct twr_radio_msg *msg, uint8_t *buf, int len)
{
	switch (msg->type) {
	case TWR_RADIO_MSG_TYPE_PUB:
		return compose_pub_msg(msg, buf, len);
	default:
		break;
	}

	return -EINVAL;
}

static int send_msg(struct twr_radio_msg *msg)
{
	static uint8_t buf[96];

	int len = compose_msg(msg, buf, sizeof(buf));
	if (len < 0) {
		return len;
	}

	LOG_HEXDUMP_DBG(buf, len, "Sending data:");

	k_sem_take(&m_tx_done_event, K_NO_WAIT);
	int ret = spirit1_tx(m_spirit1, true, buf, len);
	if (ret) {
		return ret;
	}

	// There's not really a need to customize this timeout, it's just a safety
	return k_sem_take(&m_tx_done_event, K_MSEC(1000));
}

int twr_radio_pub(struct twr_radio_msg *msg, k_timeout_t timeout)
{
	int ret;

	if (msg->type != TWR_RADIO_MSG_TYPE_PUB) {
		return -EINVAL;
	}

	if (k_mutex_lock(&m_mutex, timeout) != 0) {
		return -EBUSY;
	}

	if (msg->id == 0) {
		msg->id = m_msg_id_counter++;
	}

	k_sem_reset(&m_recv_event);

	spirit1_standby(m_spirit1);

	ret = send_msg(msg);
	if (ret) {
		goto cleanup;
	}

	if (timeout.ticks == 0) {
		goto cleanup;
	}

	struct twr_radio_msg recv;
	do {
		m_recv_msg = &recv;
		ret = spirit1_rx(m_spirit1, timeout);
		if (ret) {
			goto cleanup;
		}

		ret = k_sem_take(&m_recv_event, timeout);
		if (ret == -EAGAIN) {
			ret = -ETIMEDOUT;
			goto cleanup;
		}
	} while (m_recv_msg->id != msg->id && m_recv_msg->header != TWR_RADIO_HEADER_ACK);

cleanup:
	k_sem_reset(&m_recv_event);
	k_mutex_unlock(&m_mutex);

	return ret;
}

int twr_radio_recv(struct twr_radio_msg *msg, k_timeout_t timeout)
{
	int ret;

	if (k_mutex_lock(&m_mutex, timeout) != 0) {
		return -EBUSY;
	}

	m_recv_msg = msg;

	k_sem_take(&m_recv_event, K_NO_WAIT);

	ret = spirit1_rx(m_spirit1, timeout);
	if (ret) {
		goto cleanup;
	}

	ret = k_sem_take(&m_recv_event, timeout);
	if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
		goto cleanup;
	} else if (ret) {
		goto cleanup;
	}

cleanup:
	k_sem_reset(&m_recv_event);
	k_mutex_unlock(&m_mutex);
	return ret;
}

static int parse_pub_msg(struct twr_radio_msg *msg, uint8_t *buf, size_t len)
{
	uint8_t *ptr = buf;
	uint64_t id;

	ptr = buffer_to_radio_id(ptr, &id);
	ptr = buffer_to_msg_id(ptr, &msg->id);
	msg->header = *ptr++;
	msg->pub.data_len = len - (ptr - buf);
	memcpy(msg->pub.data, ptr, msg->pub.data_len);

	return 0;
}

static int parse_node_msg(struct twr_radio_msg *msg, uint8_t *buf, size_t len)
{
	uint8_t *ptr = buf;
	uint64_t id;

	ptr = buffer_to_radio_id(ptr, &msg->node.source_id);
	ptr = buffer_to_msg_id(ptr, &msg->id);
	msg->header = *ptr++;
	ptr = buffer_to_radio_id(ptr, &id);
	if (id != m_radio_id) {
		return 1;
	}
	msg->node.data_len = len - (ptr - buf);
	memcpy(msg->node.data, ptr, msg->node.data_len);

	return 0;
}

static int parse_msg(struct twr_radio_msg *msg, uint8_t *buf, size_t len)
{
	uint64_t id;
	buffer_to_radio_id(buf, &id);

	// If the ID is ours, the message is probably an ACK
	if (id == m_radio_id) {
		return parse_pub_msg(msg, buf, len);
	}

	return parse_node_msg(msg, buf, len);
}

static int receive_msg(uint8_t *buf, size_t len)
{
	LOG_HEXDUMP_DBG(buf, len, "Received data:");

	if (m_recv_msg == NULL) {
		k_sem_give(&m_recv_event);
		return 0;
	}

	int ret = parse_msg(m_recv_msg, buf, len);
	if (ret < 0) {
		m_recv_msg = NULL;
		LOG_ERR("Failed to parse message: %d", ret);
	} else if (ret > 0) {
		m_recv_msg = NULL;
		LOG_DBG("Message for another radio");
		return 0;
	}

	k_sem_give(&m_recv_event);

	return 0;
}

static int spirit_event_cb(enum spirit1_event event, void *user_data)
{
	int ret;

	switch (event) {
	case SPIRIT1_EVENT_RX_DATA_READY:
		LOG_DBG("SPIRIT1_EVENT_RX_DATA_READY\n");
		static uint8_t buf[96];

		ret = spirit1_get_rx_data(m_spirit1, buf, sizeof(buf));
		if (ret < 0) {
			LOG_ERR("Call `spirit1_get_rx_data` failed: %d", ret);
			return ret;
		}

		return receive_msg(buf, ret);
	case SPIRIT1_EVENT_RX_DATA_DISC:
		LOG_DBG("SPIRIT1_EVENT_RX_DATA_DISC\n");
		break;
	case SPIRIT1_EVENT_RX_TIMEOUT:
		LOG_DBG("SPIRIT1_EVENT_RX_TIMEOUT\n");
		break;
	case SPIRIT1_EVENT_TX_DATA_SENT:
		LOG_DBG("SPIRIT1_EVENT_TX_DATA_SENT\n");
		k_sem_give(&m_tx_done_event);
		break;
	case SPIRIT1_EVENT_MAX_BO_CCA_REACH:
		LOG_DBG("SPIRIT1_EVENT_MAX_BO_CCA_REACH\n");
		break;
	default:
		LOG_DBG("Unknown event\n");
		break;
	}

	return 0;
}

static int init(void)
{
	int ret;

	LOG_INF("System initialization");

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