/**
 * @file ir_isr_attr.hpp
 * @brief Placement of code that runs in RMT ISRs (private).
 */

#pragma once

#include "esp_attr.h"
#include "sdkconfig.h"

/// IRAM when CONFIG_IR_LASERTAG_ISR_IRAM_SAFE is set, flash otherwise.
#if CONFIG_IR_LASERTAG_ISR_IRAM_SAFE
#define IR_LASERTAG_ISR_ATTR IRAM_ATTR
#else
#define IR_LASERTAG_ISR_ATTR
#endif
