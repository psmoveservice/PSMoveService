//-- includes -----
#include "ServerLog.h"
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/program_options.hpp>
#include <string>

//-- globals -----
boost::log::sources::severity_logger< boost::log::trivial::severity_level > logger;
boost::log::sources::severity_logger< boost::log::trivial::severity_level > *g_logger= &logger;

//-- public implementation -----
void log_init(boost::program_options::variables_map *options)
{
    boost::log::add_common_attributes();

    boost::log::add_console_log
    (
        std::cout, boost::log::keywords::format = "[%TimeStamp%]: %Message%"
    ); 

    boost::log::add_file_log
    (
        boost::log::keywords::file_name = "PSMoveService_%N.log",     /*< file name pattern >*/
        boost::log::keywords::rotation_size = 10 * 1024 * 1024,       /*< rotate files every 10 MiB... >*/
        boost::log::keywords::format = "[%TimeStamp%]: %Message%"     /*< log record format >*/
    );

    boost::log::trivial::severity_level sev_level= boost::log::trivial::info;

    if (options->count("log_level"))
    {
        std::string log_level= (*options)["log_level"].as<std::string>();

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
    }

    boost::log::core::get()->set_filter
    (
        boost::log::trivial::severity >= sev_level
    );
}