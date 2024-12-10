#ifndef WL_INCLUDE_DRIVERS_ST_SPIRIT1_H_
#define WL_INCLUDE_DRIVERS_ST_SPIRIT1_H_

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/device.h>

/* Standard includes */
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup spirit1 spirit1
 * @{
 */

#define SPIRIT1_FIFO_SIZE 96

enum spirit1_band {
	SPIRIT1_BAND_868 = 0,
	SPIRIT1_BAND_915 = 1,
};

enum spirit1_event {
	SPIRIT1_EVENT_RX_DATA_READY,
	SPIRIT1_EVENT_RX_DATA_DISC,
	SPIRIT1_EVENT_RX_TIMEOUT,
	SPIRIT1_EVENT_TX_DATA_SENT,
	SPIRIT1_EVENT_MAX_BO_CCA_REACH,
};

typedef int (*spirit1_event_cb)(enum spirit1_event event, void *user_data);

/** @private */
typedef int (*spirit1_api_get_rx_data)(const struct device *dev, void *data, size_t length);
typedef int (*spirit1_api_set_event_cb)(const struct device *dev, spirit1_event_cb cb,
					void *user_data);
typedef int (*spirit1_api_config)(const struct device *dev, enum spirit1_band band,
				  uint8_t channel);
typedef int (*spirit1_api_rx)(const struct device *dev, k_timeout_t timeout);
typedef int (*spirit1_api_tx)(const struct device *dev, bool csma, const void *data,
			      const size_t length);
typedef int (*spirit1_api_cw)(const struct device *dev, bool enable);
/** @private */

struct spirit1_driver_api {
	spirit1_api_get_rx_data get_rx_data;
	spirit1_api_set_event_cb set_event_cb;
	spirit1_api_config config;
	spirit1_api_rx rx;
	spirit1_api_tx tx;
	spirit1_api_cw cw;
};

static inline int spirit1_get_rx_data(const struct device *dev, void *data, size_t length)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->get_rx_data(dev, data, length);
}

static inline int spirit1_set_event_cb(const struct device *dev, spirit1_event_cb cb,
				       void *user_data)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->set_event_cb(dev, cb, user_data);
}

static inline int spirit1_config(const struct device *dev, enum spirit1_band band, uint8_t channel)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->config(dev, band, channel);
}

static inline int spirit1_rx(const struct device *dev, k_timeout_t timeout)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->rx(dev, timeout);
}

static inline int spirit1_tx(const struct device *dev, bool csma, const void *data,
			     const size_t length)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->tx(dev, csma, data, length);
}

static inline int spirit1_cw(const struct device *dev, bool enable)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->cw(dev, enable);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* WL_INCLUDE_DRIVERS_ST_SPIRIT1_H_ */
