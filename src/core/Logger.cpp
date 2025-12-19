#include "mad/core/Logger.hpp"

#include <filesystem>
#include <vector>
#include <unordered_map>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

namespace mad {

namespace {

std::shared_ptr<spdlog::logger> gLogger;
bool gEnabled = true;
FileSinkConfig_t gFileConfig{};
ClassSinkConfig_t gClassConfig{};
std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> gClassLoggers;

void BuildLogger() {
  if (!gEnabled) {
    gLogger.reset();
    spdlog::drop("mad");
    return;
  }

  spdlog::drop("mad");
  gLogger.reset();

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

  if (gFileConfig.enabled) {
    std::filesystem::path logPath(gFileConfig.path);
    if (logPath.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(logPath.parent_path(), ec);
    }
    try {
      sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          gFileConfig.path, gFileConfig.maxSizeBytes, gFileConfig.maxFiles));
    } catch (const spdlog::spdlog_ex&) {
      // If file sink fails, continue with stdout.
    }
  }

  gLogger = std::make_shared<spdlog::logger>("mad", sinks.begin(), sinks.end());
  gLogger->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
  gLogger->set_level(spdlog::level::info);
  spdlog::register_logger(gLogger);
}

std::shared_ptr<spdlog::logger> BuildClassLogger(const std::string& name) {
  if (!gEnabled || !gClassConfig.enabled) {
    return nullptr;
  }
  const std::filesystem::path dir(gClassConfig.directory);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  const std::filesystem::path path = dir / (name + ".log");
  try {
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        path.string(), gClassConfig.maxSizeBytes, gClassConfig.maxFiles);
    auto logger = std::make_shared<spdlog::logger>(name, sink);
    logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    logger->set_level(spdlog::level::info);
    spdlog::register_logger(logger);
    return logger;
  } catch (const spdlog::spdlog_ex&) {
    return nullptr;
  }
}

} // namespace

void Logger::Initialize() {
  if (!gLogger && gEnabled) {
    BuildLogger();
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

std::shared_ptr<spdlog::logger> Logger::GetClass(const std::string& name) {
  if (!gEnabled || !gClassConfig.enabled) {
    return Get();
  }
  auto it = gClassLoggers.find(name);
  if (it != gClassLoggers.end()) {
    return it->second;
  }
  auto logger = BuildClassLogger(name);
  if (logger) {
    gClassLoggers[name] = logger;
    return logger;
  }
  return Get();
}

void Logger::SetEnabled(bool enabled) {
  gEnabled = enabled;
  BuildLogger();
}

void Logger::SetLevel(spdlog::level::level_enum level) {
  if (gEnabled) {
    if (!gLogger) {
      Initialize();
    }
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

void Logger::ConfigureFileSink(const FileSinkConfig_t& config) {
  gFileConfig = config;
  BuildLogger();
}

void Logger::ConfigureClassSink(const ClassSinkConfig_t& config) {
  gClassConfig = config;
  gClassLoggers.clear();
}

} // namespace mad
