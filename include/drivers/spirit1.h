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

/** @private */
typedef int (*spirit1_api_config)(const struct device *dev, enum spirit1_band band,
				  uint8_t channel);
typedef int (*spirit1_api_rx)(const struct device *dev, void *data, const size_t length,
			      k_timeout_t timeout);
typedef int (*spirit1_api_tx)(const struct device *dev, bool csma, const void *data,
			      const size_t length);
typedef int (*spirit1_api_cw)(const struct device *dev, bool enable);
/** @private */

struct spirit1_driver_api {
	spirit1_api_config config;
	spirit1_api_rx rx;
	spirit1_api_tx tx;
	spirit1_api_cw cw;
};

static inline int spirit1_config(const struct device *dev, enum spirit1_band band, uint8_t channel)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->config(dev, band, channel);
}

static inline int spirit1_rx(const struct device *dev, void *data, const size_t length,
			     k_timeout_t timeout)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->rx(dev, data, length, timeout);
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
