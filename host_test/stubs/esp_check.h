/**
 * @file esp_check.h
 * @brief ESP_RETURN_ON_FALSE stub for host-based testing.
 */

#pragma once

#include "esp_err.h"

#define ESP_RETURN_ON_FALSE(a, err_code, log_tag, ...) \
  do {                                                 \
    if (!(a)) {                                        \
      return err_code;                                 \
    }                                                  \
  } while (0)
