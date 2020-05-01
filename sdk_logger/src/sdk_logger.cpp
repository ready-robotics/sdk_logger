#include "sdk_logger/sdk_logger.hpp"

#include "sdk_logger/ros_sink.hpp"

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
    _logger = std::make_shared<spdlog::logger>("<SDK>", ros_sink);
    return _logger;
}

} // namespace sdk_logger
