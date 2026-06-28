#if !defined(CONFIG_BOARD_NATIVE_SIM)
#include "pcnt_reader.h"
#include <hal/pcnt_ll.h>

void pcnt_init_unit(uint8_t unit_idx) {
    pcnt_dev_t *hw = PCNT_LL_GET_HW(0);
    
    // 1. Stop and Clear
    pcnt_ll_stop_count(hw, unit_idx);
    pcnt_ll_clear_count(hw, unit_idx);
    
    // 2. Configure the hardware glitch filter to ignore noise pulses.
    // 1023 is the maximum filter threshold in APB clock cycles (~12.8 microseconds at 80MHz).
    pcnt_ll_set_glitch_filter_thres(hw, unit_idx, 1023);
    pcnt_ll_enable_glitch_filter(hw, unit_idx, true);
    
    // 3. Start counting
    pcnt_ll_start_count(hw, unit_idx);
}

int16_t pcnt_get_unit_count(uint8_t unit_idx) {
    return pcnt_ll_get_count(PCNT_LL_GET_HW(0), unit_idx);
}
#else
#include "pcnt_reader.h"
void pcnt_init_unit(uint8_t unit_idx) {}
int16_t pcnt_get_unit_count(uint8_t unit_idx) { return 0; }
#endif
