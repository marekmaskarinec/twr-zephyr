#ifndef ATSHA204_H_
#define ATSHA204_H_

#include <stdint.h>
#include <zephyr/device.h>

/**
 * @addtogroup atsha204 atsha204
 * @{
 */

/** @private */
typedef int (*atsha204_api_read_sn)(const struct device *dev, uint64_t *sn);
/** @private */

struct atsha204_driver_api {
	atsha204_api_read_sn read_sn;
};

static inline int atsha204_read_sn(const struct device *dev, uint64_t *sn)
{
	const struct atsha204_driver_api *api = dev->api;
	return api->read_sn(dev, sn);
}

#endif