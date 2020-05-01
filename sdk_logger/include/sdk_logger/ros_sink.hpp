#pragma once

#include <ros/ros.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>


template <typename Mutex>
class RosSink : public spdlog::sinks::base_sink<Mutex>
{
protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        spdlog::memory_buf_t formatted;
        spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
        std::string message = fmt::to_string(formatted);

        switch (msg.level) {
        case spdlog::level::level_enum::off:
            break;
        case spdlog::level::level_enum::debug:
            ROS_DEBUG("%s", message.c_str());
            break;
        case spdlog::level::level_enum::trace:
            ROS_DEBUG("%s", message.c_str());
            break;
        case spdlog::level::level_enum::info:
            ROS_INFO("%s", message.c_str());
            break;
        case spdlog::level::level_enum::warn:
            ROS_WARN("%s", message.c_str());
            break;
        case spdlog::level::level_enum::err:
            ROS_ERROR("%s", message.c_str());
            break;
        case spdlog::level::level_enum::critical:
            ROS_FATAL("%s", message.c_str());
            break;
        case spdlog::level::level_enum::n_levels:
            break;
        }
    }

    void flush_() override {}
};
