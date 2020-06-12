#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>

/*****************************************************************************/
/* Constants */
/*****************************************************************************/
/** Maximum size of the file for the file logger sink */
#define MAX_FILE_LOGGER_SIZE 1024
/** Number of files to maintain for the file logger sink */
#define MAX_FILE_LOGGER_FILES 3
/** Port to use to send data to the log service */
#define LOG_SERVICE_PORT 6000

/*****************************************************************************/
/**
 *  Main entry point
 *
 * @param[in] argc
 * @param[in] argv
 */
int main()
{
    int sock_fd;
    sockaddr_in servaddr;
    sockaddr_in clientaddr;
    int n;
    int len;
    char buffer[256];

    // Create a file logger to log all data received
    auto file_logger =
        spdlog::rotating_logger_mt("server_logger", "all_logs.txt", MAX_FILE_LOGGER_SIZE, MAX_FILE_LOGGER_FILES);
    spdlog::set_default_logger(file_logger);
    spdlog::set_pattern("%v"); // only log the string provided
    spdlog::set_level((spdlog::level::level_enum)SPDLOG_LEVEL_INFO);

    // Create the socket to recv data
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd >= 0) {
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(LOG_SERVICE_PORT);

        if (bind(sock_fd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("Bind failed");
            exit(-2);
        }
        while (true) {
            len = sizeof(clientaddr);
            n = recvfrom(
                sock_fd, buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *)&clientaddr, (socklen_t *)&len);
            buffer[n] = '\0'; // null terminate string
            if (n > 0) {
                spdlog::info(buffer);
                file_logger->flush(); // flush so that data is written to the file immediately
            }
        }
    }
    else {
        perror("Socket could not be created");
        exit(-1);
    }
}