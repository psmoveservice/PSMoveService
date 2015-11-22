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

//-- macros -----
#define SERVER_LOG_TRACE(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::trace) << function_name << " - "
#define SERVER_LOG_DEBUG(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::debug) << function_name << " - "
#define SERVER_LOG_INFO(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::info) << function_name << " - "
#define SERVER_LOG_WARNING(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::warning) << function_name << " - "
#define SERVER_LOG_ERROR(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::error) << function_name << " - "
#define SERVER_LOG_FATAL(function_name) BOOST_LOG_SEV(*g_logger, boost::log::trivial::fatal) << function_name << " - "

//-- interface -----
void log_init(boost::program_options::variables_map *options = nullptr);
 
#endif  // SERVER_LOG_H

