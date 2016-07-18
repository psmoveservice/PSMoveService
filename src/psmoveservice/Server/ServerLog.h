#ifndef SERVER_LOG_H
#define SERVER_LOG_H

//-- includes -----
#include <streambuf>
#include <ostream>
#include <iostream>
#include <mutex>

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

//-- includes -----
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

// From: http://stackoverflow.com/questions/36583873/thread-safe-access-of-stdofstream-member-using-stdmutex-member-and-operator
//###HipsterSloth $TODO: This is only really thread safe per object added to the stream
template <class char_type, class traits = std::char_traits<char_type> >
class ThreadSafeStream
{
protected:
    std::mutex mutex;
    std::basic_ostream<char_type, traits> *stream;

public:
    ThreadSafeStream(std::basic_ostream<char_type, traits> *_stream)
        : stream(_stream)
    {
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(mutex);

        stream->flush();
    }

    template<typename T>
    ThreadSafeStream<char_type>& operator<<(const T& value)
    {
        put(value);
        return *this;
    }

protected:
    template<typename T>
    void put(const T& thing) 
    {
        std::lock_guard<std::mutex> lock(mutex);

        *stream << thing;
    }
};

//-- globals -----
extern std::ostream g_normal_logger;
extern NullStream<char> g_null_logger;

extern ThreadSafeStream<char> g_mt_normal_logger;
extern ThreadSafeStream<char> g_mt_null_logger;

//-- interface -----
void log_init(const std::string &log_level);
bool log_can_emit_level(e_log_severity_level level);
std::string log_get_timestamp_prefix();

//-- macros -----
#define SELECT_LOG_STREAM(level) (log_can_emit_level(level) ? g_normal_logger : g_null_logger)
#define SELECT_MT_LOG_STREAM(level) (log_can_emit_level(level) ? g_mt_normal_logger : g_mt_null_logger)

// Non Thread Safe Logger Macros
// Almost everything is on the main thread, so you almost always want to use these
#define SERVER_LOG_TRACE(function_name) SELECT_LOG_STREAM(_log_severity_level_trace) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_LOG_DEBUG(function_name) SELECT_LOG_STREAM(_log_severity_level_debug) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_LOG_INFO(function_name) SELECT_LOG_STREAM(_log_severity_level_info) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_LOG_WARNING(function_name) SELECT_LOG_STREAM(_log_severity_level_warning) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_LOG_ERROR(function_name) SELECT_LOG_STREAM(_log_severity_level_error) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_LOG_FATAL(function_name) SELECT_LOG_STREAM(_log_severity_level_fatal) << log_get_timestamp_prefix() << function_name << " - "

// Thread Safe Logger Macros
// Uses thread safe locking before appending data to the logging stream
// Only use this when logging from other threads
#define SERVER_MT_LOG_TRACE(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_trace) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_MT_LOG_DEBUG(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_debug) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_MT_LOG_INFO(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_info) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_MT_LOG_WARNING(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_warning) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_MT_LOG_ERROR(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_error) << log_get_timestamp_prefix() << function_name << " - "
#define SERVER_MT_LOG_FATAL(function_name) SELECT_MT_LOG_STREAM(_log_severity_level_fatal) << log_get_timestamp_prefix() << function_name << " - "
 
#endif  // SERVER_LOG_H

