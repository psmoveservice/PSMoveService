#ifndef CLIENT_LOG_H
#define CLIENT_LOG_H

//-- includes -----
#include "PSMoveClient_export.h"
#include <streambuf>
#include <ostream>
#include <iostream>

//-- constants -----
enum e_log_severity_level
{
    _log_severity_level_trace,
    _log_severity_level_debug,
    _log_severity_level_info,
    _log_severity_level_warning,
    _log_severity_level_error,
    _log_severity_level_fatal
};

//-- definitions -----
// From: http://stackoverflow.com/questions/760301/implementing-a-no-op-stdostream
template <class t_char, class traits = std::char_traits<t_char> >
class NullStreamBuffer: public std::basic_streambuf<t_char, traits> 
{
    typename traits::int_type overflow(typename traits::int_type c)
    {
        return traits::not_eof(c); // indicate success
    }
};

template <class t_char, class traits = std::char_traits<t_char> >
class NullStream: public std::basic_ostream<t_char, traits> 
{
public:
    NullStream()
        : std::basic_ios<t_char, traits>(&m_sbuf)
        , std::basic_ostream<t_char, traits>(&m_sbuf)
    {
        this->init(&m_sbuf);
    }

private:
    NullStreamBuffer<t_char, traits> m_sbuf;
};

//-- externs -----
PSM_CPP_PUBLIC_CLASS extern std::ostream g_normal_logger;
PSM_CPP_PUBLIC_CLASS extern NullStream<char> g_null_logger;

//-- interface -----
PSM_CPP_PRIVATE_FUNCTION(void) log_init(e_log_severity_level level);
PSM_CPP_PUBLIC_FUNCTION(bool) log_can_emit_level(e_log_severity_level level);

//-- macros -----
#define SELECT_LOG_STREAM(level) (log_can_emit_level(level) ? g_normal_logger : g_null_logger)

#define CLIENT_LOG_TRACE(function_name) SELECT_LOG_STREAM(_log_severity_level_trace) << "[TRACE] " << function_name << " - "
#define CLIENT_LOG_DEBUG(function_name) SELECT_LOG_STREAM(_log_severity_level_debug) << "[DEBUG] " << function_name << " - "
#define CLIENT_LOG_INFO(function_name) SELECT_LOG_STREAM(_log_severity_level_info) << "[INFO] " << function_name << " - "
#define CLIENT_LOG_WARNING(function_name) SELECT_LOG_STREAM(_log_severity_level_warning) << "[WARN] " << function_name << " - "
#define CLIENT_LOG_ERROR(function_name) SELECT_LOG_STREAM(_log_severity_level_error) << "[ERROR] " << function_name << " - "
#define CLIENT_LOG_FATAL(function_name) SELECT_LOG_STREAM(_log_severity_level_fatal) << "[FATAL] " << function_name << " - "

#endif  // CLIENT_LOG_H

