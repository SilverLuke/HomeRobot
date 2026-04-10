target_sources(app PRIVATE
    rmt_tx_esp32.c 
    rmt_tx_esp32_channel.c 
)

# the esp32 module does not include the rmt hal sources.
# In the interest of keeping this module out-of-tree,
# we manually add the missing source here.
target_sources(app PRIVATE
    ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_rmt/rmt_hal.c
)

target_include_directories(app PRIVATE
    ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_rmt/include
    ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_rmt/${CONFIG_SOC_SERIES}/include
)

