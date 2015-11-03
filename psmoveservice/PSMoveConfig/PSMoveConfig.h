#include <string>
#include <boost/property_tree/ptree.hpp>

class PSMoveConfig {
public:
    PSMoveConfig(const std::string &fnamebase = std::string("PSMoveConfig"));
    void save();
    void load();
    
    std::string ConfigFileBase;

    virtual const boost::property_tree::ptree config2ptree() = 0;  // Implement by each device class' own Config
    virtual void ptree2config(const boost::property_tree::ptree &pt) = 0;  // Implement by each device class' own Config
    
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