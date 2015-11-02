#include "PSMoveConfig.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

/* System-wide data directory */
#define SYSTEM_DATA_DIR "/etc/psmoveservice"
#ifdef _WIN32
#define ENV_USER_HOME "APPDATA"
#else
#define ENV_USER_HOME "HOME"
#endif

PSMoveConfig::PSMoveConfig()
{
    boost::property_tree::ptree tree;
    //boost::property_tree::read_json(<YOUR PATH TO AND FILE NAME HERE>, tree);
    const char *homedir = getenv(ENV_USER_HOME);
#ifndef _WIN32
    // if run as root, use system-wide data directory
    if (geteuid() == 0) {
        homedir = SYSTEM_DATA_DIR;
    }
#endif
	boost::filesystem::path configpath(homedir);
	std::cout << "HomeDir: " << configpath << std::endl;
}

PSMoveConfig::~PSMoveConfig()
{
    
}