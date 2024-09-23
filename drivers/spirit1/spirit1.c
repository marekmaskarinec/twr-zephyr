
/* WL includes */
#include <drivers/spirit1.h>

/* Zephyr includes */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

/* Standard includes */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <spirit1/inc/SPIRIT_Config.h>

#define DT_DRV_COMPAT st_spirit1

LOG_MODULE_REGISTER(spirit1, CONFIG_SPIRIT1_LOG_LEVEL);

#define SPIRIT1_SPI_CFG SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE

struct config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec sdn_gpio;
};

struct data {
	struct gpio_callback irq_gpio_cb;
	struct k_spinlock spi_lock;
	struct k_mutex lock;
	struct k_event event;
	struct k_work trigger_handler_work;

	uint8_t tx_buffer[CONFIG_SPIRIT1_MAX_PACKET_SIZE];
	size_t tx_length;
};

static struct config m_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPIRIT1_SPI_CFG, 2),
	.irq_gpio = GPIO_DT_SPEC_INST_GET(0, irq_gpios),
	.sdn_gpio = GPIO_DT_SPEC_INST_GET(0, sdn_gpios),
};
static struct data m_data;

#define EVENT_TX_DONE 0x01

#define WRITE_MASK          0x00
#define READ_MASK           0x01
#define COMMAND_MASK        0x80
#define LINEAR_FIFO_ADDRESS 0xFF

static void RadioEnterShutdown(void)
{
	gpio_pin_set_dt(&m_config.sdn_gpio, 1);
	k_msleep(100);
}

static void RadioExitShutdown(void)
{
	gpio_pin_set_dt(&m_config.sdn_gpio, 0);
	k_msleep(10);
}

static StatusBytes RadioSpiRWRegisters(uint8_t mask, uint8_t cRegAddress, uint8_t cNbBytes,
				       uint8_t *pcBuffer)
{
	int ret;
	uint8_t command_hdr[2];
	StatusBytes st = {0};
	uint16_t status;
	k_spinlock_key_t key;

	command_hdr[0] = mask;
	command_hdr[1] = cRegAddress;

	const struct spi_buf tx_bufs[] = {
		{
			.buf = command_hdr,
			.len = 2,
		},
		{
			.buf = pcBuffer,
			.len = cNbBytes,
		},
	};

	const struct spi_buf rx_bufs[] = {
		{
			.buf = &status,
			.len = sizeof(status),
		},
		{
			.buf = pcBuffer,
			.len = cNbBytes,
		},
	};

	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = (mask == WRITE_MASK) ? 2 : 1};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = (mask == READ_MASK) ? 2 : 1};

	key = k_spin_lock(&m_data.spi_lock);

	ret = spi_transceive_dt(&m_config.bus, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("Call spi_transceive_dt %d", ret);
	}

	status = (status << 8) | (status >> 8);
	// LOG_INF("Status: %x", status);
	// LOG_HEXDUMP_INF(pcBuffer, cNbBytes, "Read:");
	memcpy(&st, &status, sizeof(st));

	k_spin_unlock(&m_data.spi_lock, key);

	return st;
}

StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiRWRegisters(WRITE_MASK, cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiRWRegisters(READ_MASK, cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode)
{
	return RadioSpiRWRegisters(COMMAND_MASK, cCommandCode, 0, NULL);
}

StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiRWRegisters(WRITE_MASK, LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiRWRegisters(READ_MASK, LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

static void go_to_standby(void)
{
	SpiritIrqDeInit(NULL);
	SpiritCmdStrobeSabort();
	SpiritCmdStrobeReady();
	// while (g_xStatus.MC_STATE != MC_STATE_READY) {
	// 	SpiritRefreshStatus();
	// }
	SpiritCmdStrobeStandby();
	// while (g_xStatus.MC_STATE != MC_STATE_STANDBY) {
	// 	SpiritRefreshStatus();
	// }
	SpiritIrqClearStatus();
}

static void go_to_ready()
{
	SpiritIrqDeInit(NULL);
	if (g_xStatus.MC_STATE == MC_STATE_READY) {
		return;
	}
	SpiritCmdStrobeSabort();
	SpiritCmdStrobeReady();
}

static void irq_handler(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins)
{
	gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_DISABLE);
	k_work_submit(&m_data.trigger_handler_work);
}

static void trigger_handler(struct k_work *work)
{
	SpiritIrqs irqs;
	SpiritIrqGetStatus(&irqs);

	uint8_t *pirqs = (uint8_t *)&irqs;
	LOG_INF("IRQ: 0x%02x%02x%02x%02x", pirqs[0], pirqs[1], pirqs[2], pirqs[3]);

	if (irqs.IRQ_RX_DATA_READY) {
		LOG_INF("IRQ_RX_DATA_READY");

		uint8_t buffer[96];
		uint8_t len = SpiritPktBasicGetReceivedPktLength();
		if (len > sizeof(buffer)) {
			LOG_ERR("Packet too long: %d", len);
			return;
		}

		RadioSpiReadFifo(len, buffer);

		LOG_HEXDUMP_INF(buffer, len, "RX:");

		gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		SpiritCmdStrobeFlushRxFifo();
		SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_RX_DATA_DISC) {
		LOG_INF("IRQ_RX_DATA_DISC");

		gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		SpiritCmdStrobeFlushRxFifo();
		SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_RX_TIMEOUT) {
		LOG_INF("IRQ_RX_TIMEOUT");

		gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		SpiritCmdStrobeFlushRxFifo();
		SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_TX_DATA_SENT) {
		LOG_INF("IRQ_TX_DATA_SENT");
		k_event_post(&m_data.event, EVENT_TX_DONE);
	}
}

static int enter_state_sleep()
{
	SpiritCmdStrobeSabort();
	SpiritCmdStrobeReady();
	SpiritIrqDeInit(NULL);
	SpiritIrqClearStatus();
	SpiritCmdStrobeStandby();

	return 0;
}

static int enter_state_rx()
{
	SpiritIrqs xIrqStatus;

	SpiritIrqDeInit(&xIrqStatus);
	SpiritIrq(RX_DATA_DISC, S_ENABLE);
	SpiritIrq(RX_DATA_READY, S_ENABLE);

	/* payload length config */
	SpiritPktBasicSetPayloadLength(20);

	/* enable SQI check */
	SpiritQiSetSqiThreshold(SQI_TH_0);
	SpiritQiSqiCheck(S_ENABLE);

	/* RX timeout config */
	SpiritTimerSetRxTimeoutMs(1000.0);
	SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

	SpiritRefreshStatus();
	if (g_xStatus.MC_STATE != MC_STATE_READY) {
		LOG_INF("Strobe ready");
		SpiritCmdStrobeReady();
	}

	/* IRQ registers blanking */
	SpiritIrqClearStatus();

	int ret = gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO interrupt");
		return ret;
	}

	SpiritCmdStrobeRx();

	return 0;
}

static int enter_state_tx(void)
{
	SpiritCmdStrobeSabort();
	SpiritCmdStrobeReady();
	SpiritCmdStrobeFlushTxFifo();

	SpiritIrqDeInit(NULL);
	SpiritIrqClearStatus();
	SpiritIrq(TX_DATA_SENT, S_ENABLE);

	SpiritPktBasicSetPayloadLength(m_data.tx_length);

	// TODO Why needed?
	SpiritPktBasicSetDestinationAddress(0x35);

	SpiritSpiWriteLinearFifo(m_data.tx_length, m_data.tx_buffer);

	int ret = gpio_pin_interrupt_configure_dt(&m_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO interrupt");
		return ret;
	}

	SpiritCmdStrobeTx();

	return 0;
}

static int spirit1_init(const struct device *dev)
{
	int ret;

	k_mutex_init(&m_data.lock);
	k_event_init(&m_data.event);

	k_work_init(&m_data.trigger_handler_work, trigger_handler);

	if (!spi_is_ready_dt(&m_config.bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	/* IRQ gpio */
	if (!device_is_ready(m_config.irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&m_config.irq_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO");
		return -ENODEV;
	}

	/* SDN gpio */
	if (!device_is_ready(m_config.sdn_gpio.port)) {
		LOG_ERR("SDN GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&m_config.sdn_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Cannot configure SDN GPIO");
		return -ENODEV;
	}

	SpiritRadioSetXtalFrequency(50000000);
	// SpiritSpiInit();
	SpiritEnterShutdown();
	SpiritExitShutdown();
	SpiritManagementWaExtraCurrent();

	// Config IRQ
	SGpioInit xGpioIRQ = {
		.xSpiritGpioPin = SPIRIT_GPIO_0,
		.xSpiritGpioMode = SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
		.xSpiritGpioIO = SPIRIT_GPIO_DIG_OUT_IRQ,
	};

	SpiritGpioInit(&xGpioIRQ);

	SRadioInit xRadioInit = {
		.nXtalOffsetPpm = 0,
		.lFrequencyBase = 868.0e6,
		.nChannelSpace = 20e3,
		.cChannelNumber = 0,
		.xModulationSelect = GFSK_BT1,
		.lDatarate = 19200,
		.lFreqDev = 20e3,
		.lBandwidth = 100e3,
	};
	if (SpiritRadioInit(&xRadioInit) != 0) {
		LOG_ERR("Could not init radio");
	}

	PktBasicInit xBasicInit = {
		.xPreambleLength = PKT_PREAMBLE_LENGTH_04BYTES,
		.xSyncLength = PKT_SYNC_LENGTH_4BYTES,
		.lSyncWords = 0x88888888,
		.xFixVarLength = PKT_LENGTH_VAR,
		.cPktLengthWidth = 8,
		.xCrcMode = PKT_CRC_MODE_8BITS,
		.xControlLength = PKT_CONTROL_LENGTH_0BYTES,
		.xAddressField = S_DISABLE,
		.xFec = S_DISABLE,
		.xDataWhitening = S_ENABLE,
	};

	SpiritPktBasicInit(&xBasicInit);

	PktBasicAddressesInit xAddressInit = {
		.xFilterOnMyAddress = S_DISABLE,
		.cMyAddress = 0x34,
		.xFilterOnMulticastAddress = S_DISABLE,
		.cMulticastAddress = 0xee,
		.xFilterOnBroadcastAddress = S_DISABLE,
		.cBroadcastAddress = 0xff,
	};

	SpiritPktBasicAddressesInit(&xAddressInit);

	int part_number = SpiritGeneralGetDevicePartNumber();
	int version = SpiritGeneralGetSpiritVersion();

	LOG_INF("SPIRIT1 part number: %d, version: %d", part_number, version);

	enter_state_sleep();

	// Setup IRQ callback
	gpio_init_callback(&m_data.irq_gpio_cb, irq_handler, BIT(m_config.irq_gpio.pin));
	ret = gpio_add_callback(m_config.irq_gpio.port, &m_data.irq_gpio_cb);
	if (ret) {
		LOG_ERR("Cannot add IRQ GPIO callback");
		return -ENODEV;
	}

	return 0;
}

static int api_spirit1_rx(const struct device *dev)
{
	k_mutex_lock(&m_data.lock, K_FOREVER);

	int ret = enter_state_rx();

	k_mutex_unlock(&m_data.lock);

	return ret;
}

static int api_spirit1_tx(const struct device *dev)
{
	k_mutex_lock(&m_data.lock, K_FOREVER);

	m_data.tx_length = snprintf(m_data.tx_buffer, sizeof(m_data.tx_buffer) - 1, "uptime: %u s",
				    k_uptime_seconds());
	int ret = enter_state_tx();

	k_mutex_unlock(&m_data.lock);

	return ret;
}

static const struct spirit1_driver_api spirit1_driver_api = {
	.rx = api_spirit1_rx,
	.tx = api_spirit1_tx,
};

DEVICE_DT_DEFINE(DT_DRV_INST(0), spirit1_init, NULL, &m_data, &m_config, POST_KERNEL,
		 CONFIG_SPIRIT1_INIT_PRIORITY, &spirit1_driver_api);