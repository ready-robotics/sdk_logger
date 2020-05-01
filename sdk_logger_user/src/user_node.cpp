#include <ros/ros.h>
#include <sdk_logger/sdk_logger.hpp>

#include <sstream>

struct DeathRattle
{};

namespace fmt {
template <>
struct formatter<DeathRattle> : formatter<string_view>
{
    template <typename FormatContext>
    auto format(const DeathRattle &, FormatContext &ctx) -> decltype(ctx.out())
    {
        return formatter<string_view>::format("<bleh>", ctx);
    }
};
} // namespace fmt

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_node");
    ros::NodeHandle nh;

    std::stringstream ss;
    ss << "HELLO"
       << " "
       << "world"
       << "!!!!!";

    sdk_logger::info(ss.str().c_str());
    sdk_logger::debug("GOODBYE {} EVERYTHING", 2);
    sdk_logger::warn("GAH!");
    sdk_logger::error("...I don't feel so good");
    sdk_logger::critical("{}", DeathRattle());
    sdk_logger::info("%d: %s -- %f\n\n", 2, ss.str().c_str(), 3.141);

    ros::spin();
    return 0;
}
