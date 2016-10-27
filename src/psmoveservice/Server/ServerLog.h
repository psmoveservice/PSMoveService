#ifndef SERVER_LOG_H
#define SERVER_LOG_H

//-- includes -----
#include <string>
#include <sstream>

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
class LoggerStream
{
protected:
	std::ostringstream m_lineBuffer;
	bool m_bEmitLine;

public:
	LoggerStream(bool bEmit);
	virtual ~LoggerStream();

	// accepts just about anything
	template<class T>
	LoggerStream &operator<<(const T &x)
	{
		if (m_bEmitLine)
		{
			m_lineBuffer << x;
		}

		return *this;
	}

protected:
	virtual void write_line();
};

class ThreadSafeLoggerStream : public LoggerStream
{
public:
	ThreadSafeLoggerStream(bool bEmit);

protected:
	void write_line() override;
};

//-- interface -----
void log_init(const std::string &log_level, const std::string &log_filename="");
void log_dispose();
bool log_can_emit_level(e_log_severity_level level);
std::string log_get_timestamp_prefix();

//-- macros -----
#define SELECT_LOG_STREAM(level) LoggerStream(log_can_emit_level(level))
#define SELECT_MT_LOG_STREAM(level) ThreadSafeLoggerStream(log_can_emit_level(level))

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

