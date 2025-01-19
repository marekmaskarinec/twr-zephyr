#ifndef TWR_RADIO_H_
#define TWR_RADIO_H_

/* Zephyr includes */
#include <zephyr/kernel.h>

/* Standard includes */
#include <stddef.h>

enum twr_radio_header {
	TWR_RADIO_HEADER_PAIRING = 0x00,
	TWR_RADIO_HEADER_PUB_PUSH_BUTTON = 0x01,
	TWR_RADIO_HEADER_PUB_TEMPERATURE = 0x02,
	TWR_RADIO_HEADER_PUB_HUMIDITY = 0x03,
	TWR_RADIO_HEADER_PUB_LUX_METER = 0x04,
	TWR_RADIO_HEADER_PUB_BAROMETER = 0x05,
	TWR_RADIO_HEADER_PUB_CO2 = 0x06,
	TWR_RADIO_HEADER_PUB_BUFFER = 0x07,
	TWR_RADIO_HEADER_NODE_ATTACH = 0x08,
	TWR_RADIO_HEADER_NODE_DETACH = 0x09,
	TWR_RADIO_HEADER_PUB_BATTERY = 0x0a,
	TWR_RADIO_HEADER_PUB_INFO = 0x0b, // deprecated

	TWR_RADIO_HEADER_PUB_ACCELERATION = 0x0d,
	TWR_RADIO_HEADER_PUB_TOPIC_STRING = 0x0e,
	TWR_RADIO_HEADER_PUB_TOPIC_UINT32 = 0x0f,
	TWR_RADIO_HEADER_PUB_TOPIC_BOOL = 0x10,
	TWR_RADIO_HEADER_PUB_TOPIC_INT = 0x11,
	TWR_RADIO_HEADER_PUB_TOPIC_FLOAT = 0x12,
	TWR_RADIO_HEADER_PUB_EVENT_COUNT = 0x13,
	TWR_RADIO_HEADER_PUB_STATE = 0x14,
	TWR_RADIO_HEADER_NODE_STATE_SET = 0x15,
	TWR_RADIO_HEADER_NODE_STATE_GET = 0x16,
	TWR_RADIO_HEADER_NODE_BUFFER = 0x17,
	TWR_RADIO_HEADER_NODE_LED_STRIP_COLOR_SET = 0x18,
	TWR_RADIO_HEADER_NODE_LED_STRIP_BRIGHTNESS_SET = 0x19,
	TWR_RADIO_HEADER_NODE_LED_STRIP_COMPOUND_SET = 0x1a,
	TWR_RADIO_HEADER_NODE_LED_STRIP_EFFECT_SET = 0x1b,
	TWR_RADIO_HEADER_NODE_LED_STRIP_THERMOMETER_SET = 0x1c,
	TWR_RADIO_HEADER_SUB_DATA = 0x1d,
	TWR_RADIO_HEADER_PUB_VALUE_INT = 0x1e,

	TWR_RADIO_HEADER_SUB_REG = 0x20,

	TWR_RADIO_HEADER_ACK = 0xaa,
};

enum twr_radio_msg_type {
	TWR_RADIO_MSG_TYPE_PUB = 0,
	TWR_RADIO_MSG_TYPE_NODE = 1,
	TWR_RADIO_MSG_TYPE_SUB = 2,
};

struct twr_radio_msg {
	enum twr_radio_header header;
	enum twr_radio_msg_type type;
	uint16_t id;
	union {
		struct {
			uint8_t data[51];
			int data_len;
		} pub;
		struct {
			uint64_t source_id;
			uint8_t data[45];
			int data_len;
		} node;
	};
};

int twr_radio_pub(struct twr_radio_msg *msg, k_timeout_t timeout);
int twr_radio_recv(struct twr_radio_msg *msg, k_timeout_t timeout);
void twr_radio_dump_msg(struct twr_radio_msg *msg);

#endif /* TWR_RADIO_H_ */