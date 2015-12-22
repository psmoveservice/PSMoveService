#ifndef SERVER_LOG_H
#define SERVER_LOG_H

//-- inlcludes -----
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_logger.hpp>

//-- pre-declarations -----
namespace boost {
    namespace program_options {
        class variables_map;
    }
}

//-- globals -----
extern boost::log::sources::severity_logger< boost::log::trivial::severity_level > *g_logger;
extern boost::log::sources::severity_logger_mt< boost::log::trivial::severity_level > *g_mt_logger;

//-- macros -----
// Non Thread Safe Logger Macros
// Almost everything is on the main thread, so you almost always want to use these
#define SERVER_LOG_TRACE(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::trace) << function_name << " - "
#define SERVER_LOG_DEBUG(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::debug) << function_name << " - "
#define SERVER_LOG_INFO(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::info) << function_name << " - "
#define SERVER_LOG_WARNING(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::warning) << function_name << " - "
#define SERVER_LOG_ERROR(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::error) << function_name << " - "
#define SERVER_LOG_FATAL(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::fatal) << function_name << " - "

// Thread Safe Logger Macros
// Uses thread safe locking before appending data to the logging stream
// Only use this when logging from other threads
#define SERVER_MT_LOG_TRACE(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::trace) << function_name << " - "
#define SERVER_MT_LOG_DEBUG(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::debug) << function_name << " - "
#define SERVER_MT_LOG_INFO(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::info) << function_name << " - "
#define SERVER_MT_LOG_WARNING(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::warning) << function_name << " - "
#define SERVER_MT_LOG_ERROR(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::error) << function_name << " - "
#define SERVER_MT_LOG_FATAL(function_name) BOOST_LOG_SEV(*g_mt_logger, boost::log::trivial::fatal) << function_name << " - "

//-- interface -----
void log_init(boost::program_options::variables_map *options = nullptr);
 
#endif  // SERVER_LOG_H

