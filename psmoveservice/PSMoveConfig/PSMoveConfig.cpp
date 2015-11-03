#include "PSMoveConfig.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

PSMoveConfig::PSMoveConfig(const std::string &fnamebase)
: ConfigFileBase(fnamebase)
{
}

const std::string
PSMoveConfig::getConfigPath()
{
    const char *homedir;
#ifdef _WIN32
    homedir = getenv("APPDATA");
#else
    homedir = getenv("HOME");
    // if run as root, use system-wide data directory
    if (geteuid() == 0) {
        homedir = "/etc/psmoveservice";
    }
#endif
    
    boost::filesystem::path configpath(homedir);
    configpath /= "PSMoveService";
    boost::filesystem::create_directory(configpath);
    configpath /= ConfigFileBase + ".json";
    std::cout << "Config file name: " << configpath << std::endl;
    return configpath.string();
}

void
PSMoveConfig::save()
{
    boost::property_tree::write_json(getConfigPath(), config2ptree());
}

void
PSMoveConfig::load()
{
    boost::property_tree::ptree pt;
    std::string configPath = getConfigPath();
    if ( boost::filesystem::exists( configPath ) )
    {
        boost::property_tree::read_json(configPath, pt);
        ptree2config(pt);
    }
}