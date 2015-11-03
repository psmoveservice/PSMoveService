#include "PSMoveConfig.h"
#include <iostream>

// Function definitions would go in class .cpp
class MyConfig: public PSMoveConfig
{
public:
    MyConfig(const std::string &fnamebase = "MyConfig"): PSMoveConfig(fnamebase){}
    
    int myInt = 23;
    float myFloat = 18.02f;
    std::string myString = "class string";
    struct myStructType
    {
        float x = 0.1f;
        float y = -0.2f;
        float z = 0.3f;
    } myStruct;

    virtual const boost::property_tree::ptree
    config2ptree()
    {
        boost::property_tree::ptree pt;
        pt.put("myInt", myInt);
        pt.put("myFloat", myFloat);
        pt.put("myString", myString);
        pt.put("myStruct.x", myStruct.x);
        pt.put("myStruct.y", myStruct.y);
        pt.put("myStruct.z", myStruct.z);
        
        return pt;
    }
    
    virtual void
    ptree2config(const boost::property_tree::ptree &pt)
    {
        myInt = pt.get<int>("myInt", 0);
        myFloat = pt.get<float>("myFloat", 0.0f);
        myString = pt.get<std::string>("myString", "default string");
        myStruct.x = pt.get<float>("myStruct.x", 0.0f);
        myStruct.y = pt.get<float>("myStruct.y", 0.0f);
        myStruct.z = pt.get<float>("myStruct.z", 0.0f);
    }
};

int main()
{
    MyConfig myConfig("test_config");
    std::cout << "Default myInt = " << myConfig.myInt << std::endl;
    myConfig.load();  // Note we could load() in the constructor but then we couldn't print the default value.
    std::cout << "Loaded myInt = " << myConfig.myInt << std::endl;
    myConfig.myInt = 40;
    myConfig.save();
    return 0;
    
    // Try editing the json and running again.
}