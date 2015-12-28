//-- includes -----
#include "ServerLog.h"
#include "PSMoveService.h"
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <clocale>

//-- globals -----
boost::log::sources::severity_logger< boost::log::trivial::severity_level > logger;
boost::log::sources::severity_logger< boost::log::trivial::severity_level > *g_logger= &logger;

boost::log::sources::severity_logger_mt< boost::log::trivial::severity_level > mt_logger;
boost::log::sources::severity_logger_mt< boost::log::trivial::severity_level > *g_mt_logger= &mt_logger;

//-- public implementation -----
void log_init(const std::string &log_level)
{
    std::setlocale(LC_ALL, "en_US.utf8");

    boost::log::add_common_attributes();

    boost::shared_ptr< boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > > console_sink = 
    boost::log::add_console_log
    (
        std::cout, boost::log::keywords::format = "[%TimeStamp%]: %Message%"
    );

    boost::shared_ptr< boost::log::sinks::synchronous_sink< boost::log::sinks::text_file_backend > > file_sink = 
    boost::log::add_file_log
    (
        boost::log::keywords::file_name = "PSMoveService_%N.log",     /*< file name pattern >*/
        boost::log::keywords::rotation_size = 10 * 1024 * 1024,       /*< rotate files every 10 MiB... >*/
        boost::log::keywords::format = "[%TimeStamp%]: %Message%"     /*< log record format >*/
    );

    boost::log::trivial::severity_level sev_level= boost::log::trivial::info;

    if (log_level == "trace")
    {
        sev_level= boost::log::trivial::trace;
    }
    else if (log_level == "debug")
    {
        sev_level= boost::log::trivial::debug;
    }
    else if (log_level == "info")
    {
        sev_level= boost::log::trivial::info;
    }
    else if (log_level == "warning")
    {
        sev_level= boost::log::trivial::warning;
    }
    else if (log_level == "error")
    {
        sev_level= boost::log::trivial::error;
    }
    else if (log_level == "fatal")
    {
        sev_level= boost::log::trivial::fatal;
    }

    boost::log::core::get()->set_filter
    (
        boost::log::trivial::severity >= sev_level
    );
}