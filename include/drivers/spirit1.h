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

enum spirit1_band {
	SPIRIT1_BAND_868 = 0,
	SPIRIT1_BAND_915 = 1,
};

/** @private */
typedef int (*spirit1_api_rx)(const struct device *dev);
typedef int (*spirit1_api_tx)(const struct device *dev);
/** @private */

struct spirit1_driver_api {
	spirit1_api_rx rx;
	spirit1_api_tx tx;
};

static inline int spirit1_rx(const struct device *dev)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->rx(dev);
}

static inline int spirit1_tx(const struct device *dev)
{
	const struct spirit1_driver_api *api = (const struct spirit1_driver_api *)dev->api;
	return api->tx(dev);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* WL_INCLUDE_DRIVERS_ST_SPIRIT1_H_ */
