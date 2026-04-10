# SPDX-License-Identifier: Apache-2.0
zephyr_syscall_header(${PROJECT_SOURCE_DIR}/include/zephyr/drivers/rmt_tx.h)
add_subdirectory_ifdef(CONFIG_RMT_TX_ESP32 esp32)
