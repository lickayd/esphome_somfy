#pragma once

#include <cstdint>

namespace esphome {

/// Test-controlled clock. Defined by the test translation unit so a test can
/// advance time deterministically instead of sleeping.
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);

}  // namespace esphome
