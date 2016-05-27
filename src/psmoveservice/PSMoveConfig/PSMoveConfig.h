#ifndef PSMOVE_CONFIG_H
#define PSMOVE_CONFIG_H

#include <string>
#include <boost/property_tree/ptree.hpp>

class PSMoveConfig {
public:
    PSMoveConfig(const std::string &fnamebase = std::string("PSMoveConfig"));
    void save();
    bool load();
    
    std::string ConfigFileBase;

    virtual const boost::property_tree::ptree config2ptree() = 0;  // Implement by each device class' own Config
    virtual void ptree2config(const boost::property_tree::ptree &pt) = 0;  // Implement by each device class' own Config
    
    static void writeColorPreset(
        boost::property_tree::ptree &pt,
        const char *profile_name,
        const char *color_name,
        const struct CommonHSVColorRange *colorPreset);
    static void readColorPreset(
        const boost::property_tree::ptree &pt,
        const char *profile_name,
        const char *color_name,
        struct CommonHSVColorRange *outColorPreset,
        const struct CommonHSVColorRange *defaultPreset);

private:
    const std::string getConfigPath();
};
/*
Note that PSMoveConfig is an abstract class because it has 2 pure virtual functions.
Child classes must add public member variables that store the config data,
as well as implement config2ptree and ptree2config that use pt.put() and
pt.get(), respectively, to convert between member variables and the
property tree. See tests/test_config.cpp for an example.
*/
#endif // PSMOVE_CONFIG_H