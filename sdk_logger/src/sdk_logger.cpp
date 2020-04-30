#include "sdk_logger/ros_sink.hpp"

#include <memory>
#include <mutex>

namespace sdk_logger
{

void boop()
{
    auto ros_sink = std::make_shared<RosSink<std::mutex>>();
    auto logger = std::make_shared<spdlog::logger>("ros_logger", ros_sink);
    spdlog::set_default_logger(logger);

    spdlog::info("BOOP");
}

}
