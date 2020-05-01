#include "sdk_logger/sdk_logger.hpp"

#include "sdk_logger/ros_sink.hpp"

#include <cstdio>
#include <mutex>

namespace sdk_logger {

namespace detail {
    static std::mutex _logger_mtx;
}

std::shared_ptr<logger> default_logger()
{
    static std::shared_ptr<logger> _logger{nullptr};

    std::lock_guard<std::mutex> lock(detail::_logger_mtx);
    if (_logger) {
        return _logger;
    }

    auto ros_sink = std::make_shared<RosSink<std::mutex>>();
    _logger = std::make_shared<spdlog::logger>("SDK", ros_sink);
    _logger->set_pattern("<%n> %v");
    return _logger;
}

void info(const char *format, ...)
{
    va_list args1;
    va_start(args1, format);

    va_list args2;
    va_copy(args2, args1);

    char buf[1+vsnprintf(NULL, 0, format, args1)];
    va_end(args1);

    vsnprintf(buf, sizeof(buf), format, args2);
    va_end(args2);

    default_logger()->info(buf);
}
} // namespace sdk_logger
