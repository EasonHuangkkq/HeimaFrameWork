//g++ -std=c++20 ecat_domain_probe.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp rs485.cpp -I. -o ecat_domain_probe -lpthread -lethercat
//sudo ./ecat_domain_probe --run --by-index

#include "ecat.h"
#include "config_xml.h"
#include "common.h"
#include "rs485.h"
#include <unistd.h>
#include <iostream>
#include <vector>
#include <map>
#include <atomic>

// 提供 ecat.cpp 需要的全局符号（独立于 DriverSDK 主流程）。
namespace DriverSDK{
ConfigXML* configXML = nullptr;
std::vector<std::map<int, std::string>> rs485alias2type, canAlias2type, ecatAlias2type, rs485emuAlias2type;
std::vector<std::map<int, int>> canAlias2masterID, canAlias2slaveID, ecatAlias2domain;
std::vector<std::vector<int>> ecatDomainDivision;
int dofLeg = 0, dofArm = 0, dofWaist = 0, dofNeck = 0, dofAll = 0, dofLeftEffector = 0, dofRightEffector = 0, dofEffector = 0;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers = nullptr;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** legs[2];
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** arms[2];
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** waist = nullptr;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** neck = nullptr;
WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits = nullptr;
WrapperPair<ConverterRxData, ConverterTxData, EffectorParameters> converters[2];
WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
std::vector<unsigned short> processorsECAT;
std::vector<unsigned short> maxCurrent;
std::atomic<int> ecatStalled(0);
std::vector<RS485>* rs485sPtr = nullptr;
}

using namespace DriverSDK;

static void setup_globals(std::string const& xmlPath){
    std::cout << "[probe] load config: " << xmlPath << std::endl;
    configXML = new ConfigXML(xmlPath.c_str());
    // 计算 DOF
    auto motorAlias = configXML->motorAlias();
    dofLeg   = motorAlias.size() > 0 ? motorAlias[0].size() : 0;
    dofArm   = motorAlias.size() > 2 ? motorAlias[2].size() : 0;
    dofWaist = motorAlias.size() > 4 ? motorAlias[4].size() : 0;
    dofNeck  = motorAlias.size() > 5 ? motorAlias[5].size() : 0;
    dofAll = 0;
    for(auto const& limb : motorAlias){
        dofAll += limb.size();
    }
    // 读取总线配置
    ecatAlias2type   = configXML->alias2type("ECAT");
    ecatAlias2domain = configXML->alias2attribute("ECAT", "domain");
    ecatDomainDivision = configXML->domainDivision("ECAT");
    // 分配驱动/末端
    if(dofAll > 0){
        drivers = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>[dofAll];
        if(maxCurrent.empty()){
            maxCurrent.resize(dofAll, 1000);
        }
    }
    // CPU 亲和力（与 impClass ctor 保持一致）
    processorsECAT.clear();
    int cpu = sysconf(_SC_NPROCESSORS_ONLN) - 1;
    for(int i = 0; i < 4; i++){
        processorsECAT.push_back(cpu);
    }
    // RS485 占位，避免空指针
    static std::vector<RS485> rs485sLocal;
    rs485sPtr = &rs485sLocal;
    std::cout << "[probe] dofAll=" << dofAll << " masters=" << ecatAlias2type.size() << std::endl;
}

static void dump_offsets(int masterIdx){
    if(masterIdx >= ecatAlias2domain.size()){
        return;
    }
    for(auto const& kv : ecatAlias2domain[masterIdx]){
        int alias = kv.first;
        int dom = kv.second;
        if(drivers != nullptr && alias - 1 < dofAll){
            // offset 由 DataWrapper::init 设置，存储在 WrapperPair 的 rx/tx DataWrapper 中。
            std::cout << "[probe] alias " << alias
                      << " domain " << dom
                      << " rxOff=" << drivers[alias - 1].rx.offset
                      << " txOff=" << drivers[alias - 1].tx.offset
                      << " order=" << drivers[alias - 1].order
                      << std::endl;
        }
    }
}

int main(int argc, char** argv){
    bool matchByIndex = false;
    std::string xmlPath = "config.xml";
    bool doRun = false;
    for(int i = 1; i < argc; i++){
        std::string arg = argv[i];
        if(arg == "--run" || arg == "-r"){
            doRun = true;
        }else if(arg == "--by-index"){
            matchByIndex = true;
        }else{
            xmlPath = arg;
        }
    }
    // Propagate matching mode to ECAT.
    ecatMatchByIndex = matchByIndex;
    setup_globals(xmlPath);
    std::vector<ECAT> ecats;
    for(size_t i = 0; i < ecatAlias2type.size(); i++){
        std::cout << "[probe] create ECAT master " << i << std::endl;
        ecats.emplace_back(static_cast<int>(i));
    }
    for(size_t i = 0; i < ecats.size(); i++){
        std::cout << "[probe] check master " << i << std::endl;
        int rc = ecats[i].check();
        std::cout << "  check ret=" << rc << std::endl;
    }
    for(size_t i = 0; i < ecats.size(); i++){
        std::cout << "[probe] config master " << i << std::endl;
        int rc = ecats[i].config();
        std::cout << "  config ret=" << rc << std::endl;
        if(rc == 0){
            for(size_t d = 0; d < ecats[i].domainDivision.size(); d++){
                std::cout << "  domain " << d << " size=" << ecats[i].domainSizes[d]
                          << " ptr=" << static_cast<void*>(ecats[i].domainPtrs[d]) << std::endl;
            }
            dump_offsets(static_cast<int>(i));
        }
    }
    if(doRun){
        for(size_t i = 0; i < ecats.size(); i++){
            std::cout << "[probe] run master " << i << std::endl;
            int rc = ecats[i].run();
            std::cout << "  run ret=" << rc << std::endl;
        }
        std::cout << "[probe] running... ctrl+c to stop" << std::endl;
        while(true){
            sleep(1);
        }
    }
    return 0;
}
