/*    * config_xml.h
 */

#pragma once

#include <tinyxml2.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>

namespace DriverSDK{
class ConfigXML{
public:
    char* file;
    tinyxml2::XMLDocument xmlDoc;
    ConfigXML(char const* file);
    int writeMotorParameter(int const alias, char const* parameter, float const value);
    float readMotorParameter(int const alias, char const* parameter);
    float readDeviceParameter(char const* bus, char const* type, char const* parameter);
    std::vector<std::vector<int>> motorAlias();
    std::vector<std::vector<int>> domainDivision(char const* bus);
    std::string typeAttribute(char const* bus, char const* type, char const* name);
    std::string imuAttribute(char const* name);
    int imuBaudrate();
    long canPeriod();
    std::vector<std::tuple<int, std::vector<int>, std::string>> canBus();
    std::string device(char const* bus, int const order, char const* name);
    int attribute(char const* bus, int const order, char const* name);
    bool feature(char const* bus, int const order, char const* name);
    tinyxml2::XMLElement* busDevice(char const* bus, char const* VendorID, char const* ProductCode);
    tinyxml2::XMLElement* busDevice(char const* bus, char const* type);
    std::string type(tinyxml2::XMLElement const* deviceElement);
    std::string category(char const* bus, char const* type);
    unsigned int vendorID(tinyxml2::XMLElement const* deviceElement);
    unsigned int productCode(tinyxml2::XMLElement const* deviceElement);
    std::vector<std::vector<std::string>> pdos(tinyxml2::XMLElement* const deviceElement, char const* rxtx);
    std::vector<std::string> entry(tinyxml2::XMLElement* const deviceElement, char const* object);
    std::vector<std::map<int, std::string>> alias2type(char const* bus);
    std::vector<std::map<int, int>> alias2attribute(char const* bus, char const* name);
    tinyxml2::XMLError save();
    ~ConfigXML();
};
}