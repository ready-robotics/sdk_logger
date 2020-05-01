#pragma once

#include <spdlog/spdlog.h>

#include <memory>

namespace sdk_logger {
using string_view = spdlog::string_view_t;
using logger = spdlog::logger;

std::shared_ptr<spdlog::logger> default_logger();

template <typename... Args>
inline void debug(string_view format, Args &&... args)
{
    default_logger()->debug(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void trace(string_view format, Args &&... args)
{
    default_logger()->trace(format, std::forward<Args>(args)...);
}

void info(const char *format, ...);

template <typename... Args>
inline void warn(string_view format, Args &&... args)
{
    default_logger()->warn(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(string_view format, Args &&... args)
{
    default_logger()->error(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void critical(string_view format, Args &&... args)
{
    default_logger()->critical(format, std::forward<Args>(args)...);
}
} // namespace sdk_logger
