#include "socket_sink.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

/*****************************************************************************/
/* Typedefs */
/*****************************************************************************/
/** Structure used to create threads */
typedef struct
{
    uint32_t period_ms; /**< Period of the thread, in milliseconds */
    const char *name;   /**< Name of the thread */
} thread_info_t;

/*****************************************************************************/
/* Constants */
/*****************************************************************************/
/** Command line usage string */
#define USAGE_STR \
    "Executes a sample program to utilize the spdlog module\n" \
    "-c option to enable console output logging\n" \
    "-s option enables logging to central logging server\n" \
    "-f <filename> enables logging to the provided filename\n" \
    "-x Force a seg fault to occur after 10 iterations"
/** Maximum size of the file for the file logger sink */
#define MAX_FILE_LOGGER_SIZE 1024
/** Number of files to maintain for the file logger sink */
#define MAX_FILE_LOGGER_FILES 3
/** Port to use to send data to the log service */
#define LOG_SERVICE_PORT 6000
/** Maximum depth of the backtrace to log when signal occurs */
#define MAX_BACKTRACE_SIZE 100

/*****************************************************************************/
/* Static variables */
/*****************************************************************************/
/** Boolean set based on the command line arguments to enable console port
 * logging */
static bool _console_log_enabled = false;
/** Boolean set based on the command line arguments to enable the central
 * logging sink */
static bool _server_log_enabled = false;
/** Filename to store the file logging sink. Set to NULL if no file provided */
static char *_log_file_name = NULL;
/** Log level requested on the command line */
static int32_t _log_level = SPDLOG_LEVEL_ERROR;
/** Cause a seg fault after so many iterations, when enabled on command line */
static bool _enable_seg_fault = false;

/*****************************************************************************/
/* Function prototypes */
/*****************************************************************************/
static void _force_seg_fault(void);
static void _parseArguments(int32_t argc, char *argv[]);
static void _signalHandler(int signum);
static int _msleep(long msec);
static void *_log_thread_entry(void *arg);

/*****************************************************************************/
/**
 *  Main entry point
 *
 * @param[in] argc
 * @param[in] argv
 */
int main(int argc, char *argv[])
{
    pthread_t tid[3];
    thread_info_t thread_info[3] = {{333, "Main thread"}, {500, "Thread 1"}, {400, "Thread 2"}};
    std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> stdout_sink;
    std::shared_ptr<spdlog::sinks::socket_sink_mt> server_sink;
    std::shared_ptr<spdlog::logger> default_logger;

    _parseArguments(argc, argv);

    signal(SIGINT, _signalHandler);
    signal(SIGFPE, _signalHandler);
    signal(SIGILL, _signalHandler);
    signal(SIGSEGV, _signalHandler);
    signal(SIGABRT, _signalHandler);
    signal(SIGTERM, _signalHandler);

    /* Initialize the file logger */
    if (_log_file_name != NULL) {
        try {
            file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                _log_file_name, MAX_FILE_LOGGER_SIZE, MAX_FILE_LOGGER_FILES);
        }
        catch (const spdlog::spdlog_ex &ex) {
            printf("Log to file failed %s\n", ex.what());
            abort();
        }
    }

    /* Initialize the console logger */
    if (_console_log_enabled) {
        stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    }

    /* Initialize the server logger */
    if (_server_log_enabled) {
        server_sink = std::make_shared<spdlog::sinks::socket_sink_mt>("127.0.0.1", LOG_SERVICE_PORT);
    }

    /*
     * Add the loggers as sinks
     * This seems like a crappy way of doing it, but I could not
     * figure out a better way while still using the required initializer_list type
     */
    if ((file_sink != nullptr) && (stdout_sink != nullptr) && (server_sink != nullptr)) {
        spdlog::sinks_init_list sink_list = {file_sink, stdout_sink, server_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if ((file_sink != nullptr) && (stdout_sink != nullptr)) {
        spdlog::sinks_init_list sink_list = {file_sink, stdout_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if ((stdout_sink != nullptr) && (server_sink != nullptr)) {
        spdlog::sinks_init_list sink_list = {stdout_sink, server_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if ((file_sink != nullptr) && (server_sink != nullptr)) {
        spdlog::sinks_init_list sink_list = {file_sink, server_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if (file_sink != nullptr) {
        spdlog::sinks_init_list sink_list = {file_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if (stdout_sink != nullptr) {
        spdlog::sinks_init_list sink_list = {stdout_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else if (server_sink != nullptr) {
        spdlog::sinks_init_list sink_list = {server_sink};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    else {
        spdlog::sinks_init_list sink_list = {};
        default_logger = std::make_shared<spdlog::logger>("default_logger", sink_list);
    }
    spdlog::set_default_logger(default_logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %P:%t] [%l] %v");

    /* Adjust the log level */
    spdlog::set_level((spdlog::level::level_enum)_log_level);

    /* Create threads to log at different intervals, one of which crashes */
    for (uint32_t i = 1; i < sizeof(tid) / sizeof(tid[0]); i++) {
        (void)pthread_create(&tid[i], NULL, &_log_thread_entry, &thread_info[i]);
    }

    /* Act like the first thread */
    _log_thread_entry(&thread_info[0]);
}

/*****************************************************************************/
/**
 *  Force a segmentation fault to occur.
 */
static void _force_seg_fault(void)
{
    int *x = NULL;
    *x = 0; // crash occurs
}

/*****************************************************************************/
/**
 * Parse the arguments from the command line
 *
 * @param[in] argc  Argument count
 * @param[in] argv  Argument vector
 */
static void _parseArguments(int32_t argc, char *argv[])
{
    int c;

    while ((c = getopt(argc, argv, "csf:l:x")) != -1) {
        switch (c) {
        case 's': {
            _server_log_enabled = true;
            break;
        }
        case 'c': {
            _console_log_enabled = true;
            break;
        }
        case 'f': {
            _log_file_name = optarg;
            break;
        }
        case 'l': {
            _log_level = atoi(optarg);
            break;
        }
        case 'x': {
            _enable_seg_fault = true;
            break;
        }
        case '?': {
            printf("%s", USAGE_STR);
            exit(0);
            break;
        }
        default: {
            printf("Unknown option received: %c", c);
            abort();
        }
        }
    }
}

/*****************************************************************************/
/**
 *  Signal handler callback.
 *
 *  @param[in] signum   Signal that occurred.
 */
static void _signalHandler(int signum)
{
    int j;
    int nptrs;
    void *buffer[MAX_BACKTRACE_SIZE];
    char **strings;

    printf("got signal %d\n", signum);
    if (signum != SIGINT) {
        // log backtrace since Ctrl+C was not pressed
        spdlog::critical("Signal caught {}", signum);
        nptrs = backtrace(buffer, MAX_BACKTRACE_SIZE);
        strings = backtrace_symbols(buffer, nptrs);
        if (strings != NULL) {
            /*
             * Print the call stack, but skip the first two elements since that
             * is the signal handling in libc and this function.
             *
             * The addresses provided by this stack trace can be used within gdb to
             * get the file/line number as long as -g is used a compilation flag:
             *      gdb <program_name>
             *      info line *<addr>
             */
            for (j = 2; j < nptrs; j++) {
                spdlog::critical(strings[j]);
            }
        }
    }
    // Ensure the logger has flushed to log all data
    spdlog::default_logger()->flush();
    spdlog::shutdown();
    // Exit the process, something bad happened
    exit(signum);
}

/*****************************************************************************/
/**
 * Sleep for the given number of milliseconds.
 *
 * @param[in] msec  Number of milliseconds to wait
 *
 */
static int _msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0) {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

/*****************************************************************************/
/**
 *  Thread entry point to log data.
 *
 * @param[in] arg   Pointer to a @ref thread_info_t structure for the period of this thread.
 *
 * @return NULL is always returned
 */
static void *_log_thread_entry(void *arg)
{
    thread_info_t *ti = (thread_info_t *)arg;
    uint32_t counter = 0;

    spdlog::info("Stating thread {0:s}", ti->name);

    while (true) {
        // spdlog::warn("Counter set to {}", counter);
        spdlog::log((spdlog::level::level_enum)(counter % SPDLOG_LEVEL_OFF), "Counter set to {}", counter);
        _msleep(ti->period_ms);
        counter++;

        if (_enable_seg_fault) {
            if (counter == 10) {
                _force_seg_fault();
            }
        }
    }

    return NULL;
}