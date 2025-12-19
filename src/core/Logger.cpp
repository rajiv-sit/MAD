#include "mad/core/Logger.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace mad {

namespace {

std::shared_ptr<spdlog::logger> gLogger;
bool gEnabled = true;

} // namespace

void Logger::Initialize() {
  if (!gEnabled) {
    return;
  }
  if (gLogger) {
    return;
  }
  try {
    gLogger = spdlog::stdout_color_mt("mad");
    gLogger->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    gLogger->set_level(spdlog::level::info);
  } catch (const spdlog::spdlog_ex&) {
    gLogger.reset();
  }
}

std::shared_ptr<spdlog::logger> Logger::Get() {
  if (!gEnabled) {
    return nullptr;
  }
  if (!gLogger) {
    Initialize();
  }
  return gLogger;
}

void Logger::SetEnabled(bool enabled) {
  gEnabled = enabled;
  if (!gEnabled) {
    gLogger.reset();
    spdlog::drop("mad");
  } else {
    Initialize();
  }
}

void Logger::SetLevel(spdlog::level::level_enum level) {
  if (!gLogger) {
    Initialize();
  }
  if (gLogger) {
    gLogger->set_level(level);
  }
}

spdlog::level::level_enum Logger::ParseLevel(const std::string& value) {
  if (value == "trace") {
    return spdlog::level::trace;
  }
  if (value == "debug") {
    return spdlog::level::debug;
  }
  if (value == "warn") {
    return spdlog::level::warn;
  }
  if (value == "error") {
    return spdlog::level::err;
  }
  if (value == "critical") {
    return spdlog::level::critical;
  }
  if (value == "off") {
    return spdlog::level::off;
  }
  return spdlog::level::info;
}

} // namespace mad
