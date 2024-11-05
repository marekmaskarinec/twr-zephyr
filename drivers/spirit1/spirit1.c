
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

static void RadioEnterShutdown(void);
static void RadioExitShutdown(void);

struct config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec sdn_gpio;
};

static const struct config dev_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPIRIT1_SPI_CFG, 2),
	.irq_gpio = GPIO_DT_SPEC_INST_GET(0, irq_gpios),
	.sdn_gpio = GPIO_DT_SPEC_INST_GET(0, sdn_gpios),
};

static struct spirit1_data {
	struct gpio_callback irq_gpio_cb;
	struct k_spinlock spi_lock;
	struct k_mutex lock;
	struct k_event event;
	struct k_work irg_work;
	bool csma_enabled;
	void *rx_data;
	size_t rx_length;
} dev_data;

#define EVENT_TX_DONE    BIT(0)
#define EVENT_TX_ERROR   BIT(1)
#define EVENT_RX_DONE    BIT(2)
#define EVENT_RX_ERROR   BIT(3)
#define EVENT_RX_TIMEOUT BIT(4)

static void go_to_standby(void)
{
	SpiritIrqDeInit(NULL);
	SpiritIrqClearStatus();
	if (g_xStatus.MC_STATE == MC_STATE_STANDBY) {
		return;
	}
	SpiritCmdStrobeSabort();
	SpiritCmdStrobeReady();
	// while (g_xStatus.MC_STATE != MC_STATE_READY) {
	// 	SpiritRefreshStatus();
	// }
	SpiritCmdStrobeStandby();
	// while (g_xStatus.MC_STATE != MC_STATE_STANDBY) {
	// 	SpiritRefreshStatus();
	// }
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

static void gpio_irq_handler(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins)
{
	gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_DISABLE);
	k_work_submit(&dev_data.irg_work);
}

static void irq_handler(struct k_work *work)
{
	SpiritIrqs irqs;
	SpiritIrqGetStatus(&irqs);

	uint8_t *pirqs = (uint8_t *)&irqs;
	LOG_INF("IRQ: 0x%02x%02x%02x%02x", pirqs[0], pirqs[1], pirqs[2], pirqs[3]);

	if (irqs.IRQ_RX_DATA_READY) {
		LOG_INF("IRQ_RX_DATA_READY");
		uint8_t len = SpiritPktBasicGetReceivedPktLength();
		if (len > dev_data.rx_length) {
			LOG_ERR("Packet too long: %d", len);
			k_event_post(&dev_data.event, EVENT_RX_ERROR);
			return;
		}

		RadioSpiReadFifo(len, dev_data.rx_data);
		SpiritCmdStrobeFlushRxFifo();
		dev_data.rx_length = len;
		k_event_post(&dev_data.event, EVENT_RX_DONE);

		// gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		// SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_RX_DATA_DISC) {
		LOG_INF("IRQ_RX_DATA_DISC");
		SpiritCmdStrobeFlushRxFifo();
		gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_RX_TIMEOUT) {
		LOG_INF("IRQ_RX_TIMEOUT");
		SpiritCmdStrobeFlushRxFifo();
		k_event_post(&dev_data.event, EVENT_RX_TIMEOUT);

		// gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		// SpiritCmdStrobeRx();
	}
	if (irqs.IRQ_TX_DATA_SENT) {
		LOG_INF("IRQ_TX_DATA_SENT");
		k_event_post(&dev_data.event, EVENT_TX_DONE);
	}
	if (irqs.IRQ_MAX_BO_CCA_REACH) {
		LOG_INF("IRQ_MAX_BO_CCA_REACH");
		k_event_post(&dev_data.event, EVENT_TX_ERROR);
	}
}

static int spirit1_init(const struct device *dev)
{
	int ret;

	k_mutex_init(&dev_data.lock);
	k_event_init(&dev_data.event);

	k_work_init(&dev_data.irg_work, irq_handler);

	if (!spi_is_ready_dt(&dev_config.bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	/* IRQ gpio */
	if (!device_is_ready(dev_config.irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&dev_config.irq_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO");
		return -ENODEV;
	}

	/* SDN gpio */
	if (!device_is_ready(dev_config.sdn_gpio.port)) {
		LOG_ERR("SDN GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&dev_config.sdn_gpio, GPIO_OUTPUT_LOW);
	if (ret) {
		LOG_ERR("Cannot configure SDN GPIO");
		return -ENODEV;
	}

	int part_number = SpiritGeneralGetDevicePartNumber();
	int version = SpiritGeneralGetSpiritVersion();

	LOG_INF("SPIRIT1 part number: %d, version: %d", part_number, version);

	SpiritRadioSetXtalFrequency(50000000);

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

	go_to_standby();

	// Setup IRQ callback
	gpio_init_callback(&dev_data.irq_gpio_cb, gpio_irq_handler, BIT(dev_config.irq_gpio.pin));
	ret = gpio_add_callback(dev_config.irq_gpio.port, &dev_data.irq_gpio_cb);
	if (ret) {
		LOG_ERR("Cannot add IRQ GPIO callback");
		return -ENODEV;
	}

	return 0;
}

static int api_spirit1_config(const struct device *dev, enum spirit1_band band, uint8_t channel)
{
	int ret;

	SRadioInit xRadioInit = {
		.nXtalOffsetPpm = 0,
		.nChannelSpace = 20e3,
		.cChannelNumber = channel,
		.xModulationSelect = GFSK_BT1,
		.lDatarate = 19200,
		.lFreqDev = 20e3,
		.lBandwidth = 100e3,
	};

	switch (band) {
	case SPIRIT1_BAND_868:
		xRadioInit.lFrequencyBase = 868e6;
		break;
	case SPIRIT1_BAND_915:
		xRadioInit.lFrequencyBase = 915e6;
		break;
	default:
		LOG_ERR("Unknown band: %d", band);
		return -EINVAL;
	}

	ret = SpiritRadioInit(&xRadioInit);
	if (ret) {
		LOG_ERR("Call SpiritRadioInit: %d", ret);
		return ret;
	}

	LOG_INF("After init");

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

	LOG_INF("After pkt init");

	PktBasicAddressesInit xAddressInit = {
		.xFilterOnMyAddress = S_DISABLE,
		.cMyAddress = 0x34,
		.xFilterOnMulticastAddress = S_DISABLE,
		.cMulticastAddress = 0xee,
		.xFilterOnBroadcastAddress = S_DISABLE,
		.cBroadcastAddress = 0xff,
	};

	SpiritPktBasicAddressesInit(&xAddressInit);

	LOG_INF("After addr init");

	SpiritRadioPersistenRx(S_DISABLE);
	SpiritRadioCsBlanking(S_DISABLE);

	// CSMA config (LBT)
	CsmaInit csma_init = {
		.xCsmaPersistentMode = S_DISABLE,
		.xMultiplierTbit = TBIT_TIME_64,
		.xCcaLength = TCCA_TIME_3,
		.cMaxNb = 5,
		.nBuCounterSeed = 0xFA21,
		.cBuPrescaler = 1,
	};
	SpiritCsmaInit(&csma_init);
	SpiritQiSetRssiThresholddBm(-90); // RSSI threshold for CSMA

	LOG_INF("After csma init");

	go_to_standby();

	return 0;
}

static int api_spirit1_rx(const struct device *dev, void *data, const size_t length,
			  k_timeout_t timeout)
{
	int ret = k_mutex_lock(&dev_data.lock, timeout);
	if (ret) {
		LOG_ERR("Cannot lock mutex");
		return ret;
	}

	go_to_ready();

	SpiritCmdStrobeFlushRxFifo();

	// IRQ config
	SpiritIrqs irq = {0};
	irq.IRQ_RX_DATA_READY = S_ENABLE;
	irq.IRQ_RX_DATA_DISC = S_ENABLE;
	irq.IRQ_RX_TIMEOUT = S_ENABLE;
	SpiritIrqInit(&irq);

	// payload length config
	SpiritPktBasicSetPayloadLength(20);

	// enable SQI check
	SpiritQiSetSqiThreshold(SQI_TH_0);
	SpiritQiSqiCheck(S_ENABLE);

	// RX timeout config
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		SET_INFINITE_RX_TIMEOUT();
		SpiritTimerSetRxTimeoutStopCondition(ANY_ABOVE_THRESHOLD);
	} else {
		uint32_t ms = k_ticks_to_ms_floor32(timeout.ticks);
		if (ms > 3280) {
			LOG_WRN("RX timeout too long, setting to 3280 ms");
			ms = 3280;
		}
		SpiritTimerSetRxTimeoutMs(3000);
		// SpiritTimerSetRxTimeoutMs(ms);
		SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);
	}

	if (dev_data.csma_enabled) {
		SpiritCsma(S_DISABLE);
		dev_data.csma_enabled = false;
	}

	dev_data.rx_data = data;
	dev_data.rx_length = length;
	k_event_clear(&dev_data.event, EVENT_RX_DONE | EVENT_RX_ERROR | EVENT_RX_TIMEOUT);

	// IRQ registers blanking
	SpiritIrqClearStatus();

	ret = gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO interrupt");
		return ret;
	}

	SpiritCmdStrobeRx();

	LOG_DBG("wait on event");

	uint32_t event =
		k_event_wait(&dev_data.event, EVENT_RX_DONE | EVENT_RX_ERROR | EVENT_RX_TIMEOUT,
			     false, K_FOREVER);

	go_to_standby();

	k_mutex_unlock(&dev_data.lock);

	if ((event & EVENT_RX_ERROR) != 0) {
		LOG_ERR("RX error");
		return -EIO;
	} else if ((event & EVENT_RX_TIMEOUT) != 0) {
		LOG_ERR("RX timeout");
		return -ETIMEDOUT;
	} else if (event == 0) {
		LOG_ERR("event timeout");
		return -ETIMEDOUT;
	}

	return dev_data.rx_length;
}

static int api_spirit1_tx(const struct device *dev, bool csma, const void *data,
			  const size_t length)
{
	int ret = k_mutex_lock(&dev_data.lock, K_FOREVER);
	if (ret) {
		LOG_ERR("Cannot lock mutex");
		return ret;
	}

	LOG_DBG("start");

	go_to_ready();

	// IRQ config
	SpiritIrqs irq = {0};
	irq.IRQ_TX_DATA_SENT = S_ENABLE;
	irq.IRQ_MAX_BO_CCA_REACH = S_ENABLE;
	SpiritIrqInit(&irq);

	if (csma) {
		if (!dev_data.csma_enabled) {
			SpiritCsma(S_ENABLE);
			dev_data.csma_enabled = true;
		}
	} else {
		if (dev_data.csma_enabled) {
			SpiritCsma(S_DISABLE);
			dev_data.csma_enabled = false;
		}
	}

	// Prepare send
	SpiritPktBasicSetDestinationAddress(0x35);
	SpiritPktBasicSetPayloadLength(length);

	// Add data to FIFO
	SpiritCmdStrobeFlushTxFifo();
	RadioSpiWriteFifo(length, (uint8_t *)data);

	k_event_clear(&dev_data.event, EVENT_TX_DONE | EVENT_TX_ERROR);

	// IRQ registers blanking
	SpiritIrqClearStatus();

	ret = gpio_pin_interrupt_configure_dt(&dev_config.irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Cannot configure IRQ GPIO interrupt");
		return ret;
	}

	SpiritCmdStrobeTx();

	LOG_DBG("wait on event");

	uint32_t event =
		k_event_wait(&dev_data.event, EVENT_TX_DONE | EVENT_TX_ERROR, false, K_MSEC(500));

	go_to_standby();

	k_mutex_unlock(&dev_data.lock);

	if ((event & EVENT_TX_ERROR) != 0) {
		LOG_ERR("TX error");
		return -EIO;
	} else if (event == 0) {
		LOG_ERR("TX timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static int api_spirit1_cw(const struct device *dev, bool enable)
{
	k_mutex_lock(&dev_data.lock, K_FOREVER);

	if (enable) {
		go_to_ready();
		SpiritDirectRfSetTxMode(PN9_TX_MODE);
		SpiritRadioCWTransmitMode(S_ENABLE);
		SpiritCmdStrobeTx();
	} else {
		SpiritCmdStrobeSabort();
		SpiritDirectRfSetTxMode(NORMAL_TX_MODE);
		SpiritRadioCWTransmitMode(S_DISABLE);
		go_to_standby();
	}

	k_mutex_unlock(&dev_data.lock);

	return 0;
}

static const struct spirit1_driver_api spirit1_driver_api = {
	.config = api_spirit1_config,
	.rx = api_spirit1_rx,
	.tx = api_spirit1_tx,
	.cw = api_spirit1_cw,
};

DEVICE_DT_INST_DEFINE(0, spirit1_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_SPIRIT1_INIT_PRIORITY,
		      &spirit1_driver_api);

#define WRITE_MASK          0x00
#define READ_MASK           0x01
#define COMMAND_MASK        0x80
#define LINEAR_FIFO_ADDRESS 0xFF

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

	key = k_spin_lock(&dev_data.spi_lock);

	ret = spi_transceive_dt(&dev_config.bus, &tx, &rx);
	if (ret) {
		LOG_ERR("Call spi_transceive_dt");
	}

	status = (status << 8) | (status >> 8);
	memcpy(&st, &status, sizeof(st));

	k_spin_unlock(&dev_data.spi_lock, key);

	return st;
}

static void RadioEnterShutdown(void)
{
	gpio_pin_set_dt(&dev_config.sdn_gpio, 1);
	k_usleep(500);
}

static void RadioExitShutdown(void)
{
	gpio_pin_set_dt(&dev_config.sdn_gpio, 0);
	k_msleep(1);
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
