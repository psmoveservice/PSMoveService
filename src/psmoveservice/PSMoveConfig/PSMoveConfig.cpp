#include "PSMoveConfig.h"
#include "DeviceInterface.h"
#include "ServerUtility.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

// Format: {hue center, hue range}, {sat center, sat range}, {val center, val range}
// All hue angles are 60 degrees apart to maximize hue separation for 6 max tracked colors.
// Hue angle reference: http://i.imgur.com/PKjgfFXm.jpg 
// Hue angles divide by 2 for opencv which remaps hue range to [0,180]
const CommonHSVColorRange g_default_color_presets[] = {
    { { 300 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Magenta
    { { 180 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Cyan
    { { 60 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Yellow
    { { 0, 10 }, { 255, 32 }, { 255, 32 } }, // Red
    { { 120 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Green
    { { 240 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Blue
};
const CommonHSVColorRange *k_default_color_presets = g_default_color_presets;

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

void
PSMoveConfig::writeColorPropertyPresetTable(
	const CommonHSVColorRangeTable *table,
    boost::property_tree::ptree &pt)
{
	const char *profile_name= table->table_name.c_str();

    writeColorPreset(pt, profile_name, "magenta", &table->color_presets[eCommonTrackingColorID::Magenta]);
    writeColorPreset(pt, profile_name, "cyan", &table->color_presets[eCommonTrackingColorID::Cyan]);
    writeColorPreset(pt, profile_name, "yellow", &table->color_presets[eCommonTrackingColorID::Yellow]);
    writeColorPreset(pt, profile_name, "red", &table->color_presets[eCommonTrackingColorID::Red]);
    writeColorPreset(pt, profile_name, "green", &table->color_presets[eCommonTrackingColorID::Green]);
    writeColorPreset(pt, profile_name, "blue", &table->color_presets[eCommonTrackingColorID::Blue]);
}

void
PSMoveConfig::readColorPropertyPresetTable(
	const boost::property_tree::ptree &pt,
	CommonHSVColorRangeTable *table)
{
	const char *profile_name= table->table_name.c_str();

    readColorPreset(pt, profile_name, "magenta", &table->color_presets[eCommonTrackingColorID::Magenta], &k_default_color_presets[eCommonTrackingColorID::Magenta]);
    readColorPreset(pt, profile_name, "cyan", &table->color_presets[eCommonTrackingColorID::Cyan], &k_default_color_presets[eCommonTrackingColorID::Cyan]);
    readColorPreset(pt, profile_name, "yellow", &table->color_presets[eCommonTrackingColorID::Yellow], &k_default_color_presets[eCommonTrackingColorID::Yellow]);
    readColorPreset(pt, profile_name, "red", &table->color_presets[eCommonTrackingColorID::Red], &k_default_color_presets[eCommonTrackingColorID::Red]);
    readColorPreset(pt, profile_name, "green", &table->color_presets[eCommonTrackingColorID::Green], &k_default_color_presets[eCommonTrackingColorID::Green]);
    readColorPreset(pt, profile_name, "blue", &table->color_presets[eCommonTrackingColorID::Blue], &k_default_color_presets[eCommonTrackingColorID::Blue]);
}

void
PSMoveConfig::writeTrackingColor(
	boost::property_tree::ptree &pt,
	int tracking_color_id)
{
	switch (tracking_color_id)
	{
	case eCommonTrackingColorID::INVALID_COLOR:
		pt.put("tracking_color", "invalid");
		break;
	case eCommonTrackingColorID::Magenta:
		pt.put("tracking_color", "magenta");
		break;
	case eCommonTrackingColorID::Cyan:
		pt.put("tracking_color", "cyan");
		break;
	case eCommonTrackingColorID::Yellow:
		pt.put("tracking_color", "yellow");
		break;
	case eCommonTrackingColorID::Red:
		pt.put("tracking_color", "red");
		break;
	case eCommonTrackingColorID::Green:
		pt.put("tracking_color", "green");
		break;
	case eCommonTrackingColorID::Blue:
		pt.put("tracking_color", "blue");
		break;
	default:
		assert(false && "unreachable");
	}
}

int 
PSMoveConfig::readTrackingColor(
	const boost::property_tree::ptree &pt)
{
	std::string tracking_color_string = pt.get<std::string>("tracking_color", "invalid");
	int tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;

	if (tracking_color_string == "magenta")
	{
		tracking_color_id = eCommonTrackingColorID::Magenta;
	}
	else if (tracking_color_string == "cyan")
	{
		tracking_color_id = eCommonTrackingColorID::Cyan;
	}
	else if (tracking_color_string == "yellow")
	{
		tracking_color_id = eCommonTrackingColorID::Yellow;
	}
	else if (tracking_color_string == "red")
	{
		tracking_color_id = eCommonTrackingColorID::Red;
	}
	else if (tracking_color_string == "green")
	{
		tracking_color_id = eCommonTrackingColorID::Green;
	}
	else if (tracking_color_string == "blue")
	{
		tracking_color_id = eCommonTrackingColorID::Blue;
	}

	return tracking_color_id;
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