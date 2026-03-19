/*
 * Custom RMT TX Driver API for Zephyr
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RMT_TX_H_
#define ZEPHYR_INCLUDE_DRIVERS_RMT_TX_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RMT_TX_CARRIER_LEVEL_LOW = 0,
    RMT_TX_CARRIER_LEVEL_HIGH = 1,
} rmt_tx_carrier_level_t;

struct rmt_symbol {
    uint16_t duration0 : 15;
    uint16_t level0 : 1;
    uint16_t duration1 : 15;
    uint16_t level1 : 1;
};

typedef int (*rmt_tx_api_set_carrier)(const struct device *dev, bool carrier_en,
                                      k_timeout_t high_duration, k_timeout_t low_duration,
                                      rmt_tx_carrier_level_t carrier_level);

typedef int (*rmt_tx_api_transmit)(const struct device *dev, const struct rmt_symbol *symbols,
                                   size_t num_symbols, k_timeout_t timeout);

struct rmt_tx_driver_api {
    rmt_tx_api_set_carrier set_carrier;
    rmt_tx_api_transmit transmit;
};

static inline int rmt_tx_set_carrier(const struct device *dev, bool carrier_en,
                                     k_timeout_t high_duration, k_timeout_t low_duration,
                                     rmt_tx_carrier_level_t carrier_level)
{
    const struct rmt_tx_driver_api *api = (const struct rmt_tx_driver_api *)dev->api;
    return api->set_carrier(dev, carrier_en, high_duration, low_duration, carrier_level);
}

static inline int rmt_tx_transmit(const struct device *dev, const struct rmt_symbol *symbols,
                                  size_t num_symbols, k_timeout_t timeout)
{
    const struct rmt_tx_driver_api *api = (const struct rmt_tx_driver_api *)dev->api;
    return api->transmit(dev, symbols, num_symbols, timeout);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RMT_TX_H_ */
