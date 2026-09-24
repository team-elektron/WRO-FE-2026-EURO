#pragma once
#include <stdint.h>

#define COUNTS_PER_REV  200 //375

void     encoder_init();
void     encoder_reset();
int32_t  encoder_get_ticks();
float    encoder_get_rpm(float dt_s);   // always positive magnitude