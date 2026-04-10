#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rmt_tx.h>
#include <zephyr/logging/log.h>

#include <hal/rmt_hal.h>
#include <hal/rmt_ll.h>
#include <hal/rmt_types.h>
#include <soc/soc_caps.h>

LOG_MODULE_DECLARE(rmt_tx_esp32, CONFIG_RMT_TX_LOG_LEVEL);

#define DT_DRV_COMPAT espressif_esp32_rmt_tx

struct rmt_tx_esp32_data
{
    rmt_hal_context_t hal;
};

struct rmt_tx_esp32_channel_config
{
    const struct device *parent;
    const struct gpio_dt_spec gpio;
    uint8_t channel;
    bool invert_out;
};

static int rmt_tx_esp32_set_carrier(const struct device *dev, bool carrier_en,
                                    k_timeout_t high_duration, k_timeout_t low_duration,
                                    rmt_tx_carrier_level_t carrier_level)
{
    return -ENOTSUP;
}

static int rmt_tx_esp32_transmit(const struct device *dev, const struct rmt_symbol *symbols,
                                 size_t num_symbols, k_timeout_t timeout)
{
    const struct rmt_tx_esp32_channel_config *config = dev->config;
    struct rmt_tx_esp32_data *parent_data = (struct rmt_tx_esp32_data *)config->parent->data;
    rmt_hal_context_t *hal = &parent_data->hal;
    rmt_dev_t *rmt_dev = (rmt_dev_t *)hal->regs;
    
    // RAM base calculation
#if defined(CONFIG_SOC_ESP32C6)
    volatile rmt_symbol_word_t *ram = (volatile rmt_symbol_word_t *)((uint8_t *)hal->regs + 0x400);
#elif defined(CONFIG_SOC_ESP32S3)
    volatile rmt_symbol_word_t *ram = (volatile rmt_symbol_word_t *)((uint8_t *)hal->regs + 0x800);
#else
    #error "Unsupported SOC for RMT RAM offset"
#endif
    ram += config->channel * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    
    uint32_t event_mask = RMT_LL_EVENT_TX_DONE(config->channel);

    // 1. Stop any ongoing transmission and clear interrupts
    rmt_ll_tx_stop(rmt_dev, config->channel);
    rmt_ll_tx_reset_pointer(rmt_dev, config->channel);
    rmt_ll_clear_interrupt_status(rmt_dev, event_mask | RMT_LL_EVENT_TX_ERROR(config->channel));
    
    // 2. Copy symbols to RMT RAM using 32-bit writes
    for (size_t i = 0; i < num_symbols; i++) {
        rmt_symbol_word_t symbol;
        symbol.duration0 = symbols[i].duration0;
        symbol.level0 = symbols[i].level0;
        symbol.duration1 = symbols[i].duration1;
        symbol.level1 = symbols[i].level1;
        ram[i] = symbol;
    }
    
    // 3. Add an EOF symbol (duration = 0)
    ram[num_symbols].val = 0;

    // 4. Update configuration and start transmission
    rmt_ll_tx_start(rmt_dev, config->channel);

    // 5. Wait for completion
    uint32_t start_time = k_uptime_get_32();
    uint32_t timeout_ms = K_TIMEOUT_EQ(timeout, K_FOREVER) ? UINT32_MAX : k_ticks_to_ms_near32(timeout.ticks);
    
    while (1) {
        uint32_t status = rmt_ll_tx_get_interrupt_status_raw(rmt_dev, config->channel);
        if (status & event_mask) {
            break;
        }
        if (status & RMT_LL_EVENT_TX_ERROR(config->channel)) {
            LOG_ERR("RMT TX error on channel %d", config->channel);
            return -EIO;
        }
        if (timeout_ms != UINT32_MAX && (k_uptime_get_32() - start_time > timeout_ms)) {
            LOG_ERR("RMT TX timeout on channel %d", config->channel);
            return -ETIMEDOUT;
        }
        k_yield();
    }
    
    // Clear the interrupt status
    rmt_ll_clear_interrupt_status(rmt_dev, event_mask);

    return 0;
}

static struct rmt_tx_driver_api rmt_tx_esp32_driver_api = {
    .set_carrier = rmt_tx_esp32_set_carrier,
    .transmit = rmt_tx_esp32_transmit,
};

static int rmt_tx_esp32_channel_init(const struct device *dev)
{
    const struct rmt_tx_esp32_channel_config *config = dev->config;
    struct rmt_tx_esp32_data *parent_data = (struct rmt_tx_esp32_data *)config->parent->data;
    rmt_dev_t *rmt_dev = (rmt_dev_t *)parent_data->hal.regs;

    // parent device handles the initialization of the overall RMT peripheral
    if (!device_is_ready(config->parent))
    {
        LOG_ERR("Parent device not ready");
        return -ENODEV;
    }

    // Configure memory blocks for this channel (default to 1)
    rmt_ll_tx_set_mem_blocks(rmt_dev, config->channel, 1);
    
    // Configure channel clock divider (default to 1)
    rmt_ll_tx_set_channel_clock_div(rmt_dev, config->channel, 1);

    // Disable wrap mode, loop mode, and carrier modulation
    rmt_ll_tx_enable_wrap(rmt_dev, config->channel, false);
    rmt_ll_tx_enable_loop(rmt_dev, config->channel, false);
    rmt_ll_tx_enable_carrier_modulation(rmt_dev, config->channel, false);
    
    // Fix idle level to LOW (0) and enable it
    // level=0, enable=true
    rmt_ll_tx_fix_idle_level(rmt_dev, config->channel, 0, true);
    
    // Reset pointers and apply configuration
    rmt_ll_tx_reset_pointer(rmt_dev, config->channel);
    rmt_dev->chnconf0[config->channel].conf_update_chn = 1;

    return 0;
}

#define RMT_TX_ESP32_CHANNEL_DEFINE(node)                                        \
    BUILD_ASSERT(DT_REG_ADDR(node) < 4);                                         \
    static const struct rmt_tx_esp32_channel_config config##node = {             \
        .parent = DEVICE_DT_GET(DT_PARENT(node)),                                \
        .gpio = GPIO_DT_SPEC_GET(node, gpios),                                   \
        .channel = DT_REG_ADDR(node),                                            \
        .invert_out = DT_PROP(node, invert_out),                                 \
    };                                                                           \
    DEVICE_DT_DEFINE(node, rmt_tx_esp32_channel_init, NULL, NULL, &config##node, \
                     POST_KERNEL, CONFIG_RMT_TX_CHANNEL_INIT_PRIORITY,           \
                     &rmt_tx_esp32_driver_api);

#define RMT_TX_ESP32_CHANNEL_DEFINE_ALL(inst) \
    DT_FOREACH_CHILD(DT_DRV_INST(inst), RMT_TX_ESP32_CHANNEL_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(RMT_TX_ESP32_CHANNEL_DEFINE_ALL)
