#include "pcnt_reader.h"
#include <hal/pcnt_ll.h>

int16_t pcnt_get_unit_count(uint8_t unit_idx) {
    // Standard ESP32 PCNT hardware address
    return pcnt_ll_get_count(PCNT_LL_GET_HW(0), unit_idx);
}
