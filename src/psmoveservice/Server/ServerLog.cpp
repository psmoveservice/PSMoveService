//-- includes -----
#include "ServerLog.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

//-- globals -----
e_log_severity_level g_min_log_level;


// Connect the normal logger to standard output
std::ostream g_normal_logger(std::cout.rdbuf());

// Null stream eats the log
NullStream<char> g_null_logger;

ThreadSafeStream<char> g_mt_normal_logger(&g_normal_logger);
ThreadSafeStream<char> g_mt_null_logger(&g_null_logger);

//-- public implementation -----
void log_init(const std::string &log_level)
{
    g_min_log_level= _log_severity_level_info;

    if (log_level == "trace")
    {
        g_min_log_level= _log_severity_level_trace;
    }
    else if (log_level == "debug")
    {
        g_min_log_level= _log_severity_level_debug;
    }
    else if (log_level == "info")
    {
        g_min_log_level= _log_severity_level_info;
    }
    else if (log_level == "warning")
    {
        g_min_log_level= _log_severity_level_warning;
    }
    else if (log_level == "error")
    {
        g_min_log_level= _log_severity_level_error;
    }
    else if (log_level == "fatal")
    {
        g_min_log_level= _log_severity_level_fatal;
    }
}

bool log_can_emit_level(e_log_severity_level level)
{
    return (level >= g_min_log_level);
}

std::string log_get_timestamp_prefix()
{
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds);
    time_t in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << milliseconds.count() << "]: ";

    return ss.str();
}