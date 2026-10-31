/**
 * @file freertos/FreeRTOS.h
 * @brief Minimal FreeRTOS stub for host-based testing.
 */

#pragma once

#include <cstdint>

typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef int BaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

// Spinlock type for critical sections (no-op on the host).
typedef struct {
  int unused;
} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
