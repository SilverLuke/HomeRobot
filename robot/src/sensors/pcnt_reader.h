#ifndef PCNT_READER_H
#define PCNT_READER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pcnt_init_unit(uint8_t unit_idx);
int16_t pcnt_get_unit_count(uint8_t unit_idx);

#ifdef __cplusplus
}
#endif

#endif
