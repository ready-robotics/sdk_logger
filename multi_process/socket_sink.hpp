#include "spdlog/sinks/base_sink.h"
#include "spdlog/spdlog.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace spdlog { namespace sinks {

    template <typename Mutex>
    class socket_sink : public spdlog::sinks::base_sink<Mutex>
    {
    public:
        socket_sink(const char *to_addr, uint16_t port)
        {
            _sockFD = socket(AF_INET, SOCK_DGRAM, 0);
            memset((char *)&_serverAddr, 0, sizeof(_serverAddr));
            _serverAddr.sin_family = AF_INET;
            _serverAddr.sin_port = htons(port);
            inet_pton(AF_INET, to_addr, &_serverAddr.sin_addr);
        }

    protected:
        void sink_it_(const spdlog::details::log_msg &msg) override
        {
            // log_msg is a struct containing the log entry info like level, timestamp, thread id etc.
            // msg.raw contains pre formatted log

            // If needed (very likely but not mandatory), the sink formats the message before sending it to its final
            // destination:
            spdlog::memory_buf_t formatted;
            spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
            std::basic_string<char> s = fmt::to_string(formatted);
            if (_sockFD > 0) {
                sendto(_sockFD, s.c_str(), s.length() - 1, 0, (const sockaddr *)&_serverAddr, sizeof(_serverAddr));
            }
        }

        void flush_() override
        {}

    private:
        int _sockFD;                    /**< Socket object */
        struct sockaddr_in _serverAddr; /**< Server address object to send data */
    };

#include "spdlog/details/null_mutex.h"

#include <mutex>
    using socket_sink_mt = socket_sink<std::mutex>;
    using socket_sink_st = socket_sink<spdlog::details::null_mutex>;

}} // namespace spdlog::sinks