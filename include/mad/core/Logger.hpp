#pragma once

#include <string>

#include <memory>

#include <spdlog/logger.h>

namespace mad {

class Logger {
 public:
  static void Initialize();
  static std::shared_ptr<spdlog::logger> Get();
  static void SetEnabled(bool enabled);
  static void SetLevel(spdlog::level::level_enum level);
  static spdlog::level::level_enum ParseLevel(const std::string& value);
};

} // namespace mad
