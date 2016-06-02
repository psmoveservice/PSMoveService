#include "PSMoveConfig.h"
#include "DeviceInterface.h"
#include "ServerUtility.h"
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
    size_t homedir_buffer_req_size;
    char homedir_buffer[512];
    getenv_s(&homedir_buffer_req_size, homedir_buffer, "APPDATA");
    assert(homedir_buffer_req_size <= sizeof(homedir_buffer));
    homedir= homedir_buffer;
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

bool
PSMoveConfig::load()
{
    bool bLoadedOk = false;
    boost::property_tree::ptree pt;
    std::string configPath = getConfigPath();

    if ( boost::filesystem::exists( configPath ) )
    {
        boost::property_tree::read_json(configPath, pt);
        ptree2config(pt);
        bLoadedOk = true;
    }

    return bLoadedOk;
}

static void
writeColorPropertyPreset(
    boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const char *property_name,
    float value)
{
    char full_property_name[256];

    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "%s.color_preset.%s.%s", 
            profile_name, color_name, property_name);
    }
    else
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "color_preset.%s.%s", 
            color_name, property_name);
    }
    pt.put(full_property_name, value);
}

void
PSMoveConfig::writeColorPreset(
    boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const CommonHSVColorRange *colorPreset)
{
    writeColorPropertyPreset(pt, profile_name, color_name, "hue_center", colorPreset->hue_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "hue_range", colorPreset->hue_range.range);
    writeColorPropertyPreset(pt, profile_name, color_name, "saturation_center", colorPreset->saturation_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "saturation_range", colorPreset->saturation_range.range);
    writeColorPropertyPreset(pt, profile_name, color_name, "value_center", colorPreset->value_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "value_range", colorPreset->value_range.range);
}


static void
readColorPropertyPreset(
    const boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const char *property_name,
    float &out_value,
    const float default_value)
{
    char full_property_name[256];

    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "%s.color_preset.%s.%s", 
            profile_name, color_name, property_name);
    }
    else
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "color_preset.%s.%s",
            color_name, property_name);
    }
    out_value = pt.get<float>(full_property_name, default_value);
}

void
PSMoveConfig::readColorPreset(
    const boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    CommonHSVColorRange *outColorPreset,
    const CommonHSVColorRange *defaultPreset)
{
    readColorPropertyPreset(pt, profile_name, color_name, "hue_center", outColorPreset->hue_range.center, defaultPreset->hue_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "hue_range", outColorPreset->hue_range.range, defaultPreset->hue_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_center", outColorPreset->saturation_range.center, defaultPreset->saturation_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_range", outColorPreset->saturation_range.range, defaultPreset->saturation_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "value_center", outColorPreset->value_range.center, defaultPreset->value_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "value_range", outColorPreset->value_range.range, defaultPreset->value_range.range);
}