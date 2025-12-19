#pragma once

#include <string>
#include <cstddef>

#include <memory>
#include <string>

#include <spdlog/logger.h>

namespace mad {

struct FileSinkConfig_t {
  bool enabled = false;
  std::string path = "logs/mad.log";
  std::size_t maxSizeBytes = 5 * 1024 * 1024;
  std::size_t maxFiles = 3;
};

struct ClassSinkConfig_t {
  bool enabled = false;
  std::string directory = "logs/classes";
  std::size_t maxSizeBytes = 5 * 1024 * 1024;
  std::size_t maxFiles = 3;
};

class Logger {
 public:
  static void Initialize();
  static std::shared_ptr<spdlog::logger> Get();
  static std::shared_ptr<spdlog::logger> GetClass(const std::string& name);
  static void SetEnabled(bool enabled);
  static void SetLevel(spdlog::level::level_enum level);
  static spdlog::level::level_enum ParseLevel(const std::string& value);
  static void ConfigureFileSink(const FileSinkConfig_t& config);
  static void ConfigureClassSink(const ClassSinkConfig_t& config);
};

} // namespace mad
