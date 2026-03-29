#include "pcnt_reader.h"
#include <hal/pcnt_ll.h>

void pcnt_init_unit(uint8_t unit_idx) {
    pcnt_dev_t *hw = PCNT_LL_GET_HW(0);
    
    // 1. Stop and Clear
    pcnt_ll_stop_count(hw, unit_idx);
    pcnt_ll_clear_count(hw, unit_idx);
    
    // 2. We assume the devicetree already configured the filters and modes.
    // We just ensure it is started.
    pcnt_ll_start_count(hw, unit_idx);
}

int16_t pcnt_get_unit_count(uint8_t unit_idx) {
    return pcnt_ll_get_count(PCNT_LL_GET_HW(0), unit_idx);
}
