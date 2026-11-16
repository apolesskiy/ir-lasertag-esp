/**
 * @file esp_heap_caps.h
 * @brief Heap capability stub for host-based testing.
 */

#pragma once

#include <cstdlib>

#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_8BIT (1 << 2)

inline void* heap_caps_calloc(size_t n, size_t size, uint32_t) {
  return calloc(n, size);
}
inline void* heap_caps_malloc(size_t size, uint32_t) { return malloc(size); }
