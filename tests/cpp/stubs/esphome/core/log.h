#pragma once

#include <cinttypes>
#include <cstdarg>
#include <cstdio>

// Logging that still *consumes* its arguments, so variables and TAGs that exist
// only for logging do not produce unused-variable warnings when the real
// component sources are compiled against these stubs. It is a no-op unless a
// suite installs a hook to assert on the formatted lines.
namespace esphome {
using LogHook = void (*)(char level, const char *tag, const char *message);
inline LogHook &log_hook() {
  static LogHook hook = nullptr;
  return hook;
}
inline void log_sink(char level, const char *tag, const char *format, ...) {
  if (log_hook() == nullptr)
    return;
  char message[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);
  log_hook()(level, tag, message);
}
}  // namespace esphome

#define ESP_LOGD(tag, ...) ::esphome::log_sink('D', tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) ::esphome::log_sink('I', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) ::esphome::log_sink('W', tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) ::esphome::log_sink('E', tag, __VA_ARGS__)
#define ESP_LOGV(tag, ...) ::esphome::log_sink('V', tag, __VA_ARGS__)
#define ESP_LOGCONFIG(tag, ...) ::esphome::log_sink('C', tag, __VA_ARGS__)
