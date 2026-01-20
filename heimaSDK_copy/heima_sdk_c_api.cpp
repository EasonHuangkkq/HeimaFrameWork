/*
 * heima_sdk_c_api.cpp
 *
 * Minimal C ABI around DriverSDK::DriverSDK for Python ctypes usage.
 */

#include "heima_sdk_c_api.h"

#include "heima_driver_sdk.h"

#include <mutex>
#include <string>
#include <vector>

static_assert(sizeof(HeimaImu) == 9 * sizeof(float), "HeimaImu ABI mismatch");
static_assert(sizeof(HeimaMotorActual) == 20, "HeimaMotorActual ABI mismatch");

namespace{
std::mutex gMutex;
bool gInitialized = false;
int gMotorNr = 0;
std::string gVersion;
}

extern "C"{

int heima_sdk_init(char const* config_xml_path){
    if(config_xml_path == nullptr){
        return -1;
    }
    std::lock_guard<std::mutex> lock(gMutex);
    if(gInitialized){
        return 0;
    }
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(config_xml_path);
    gMotorNr = sdk.getTotalMotorNr();
    gVersion = sdk.version();
    gInitialized = true;
    return 0;
}

int heima_sdk_get_total_motor_nr(void){
    std::lock_guard<std::mutex> lock(gMutex);
    if(!gInitialized){
        return -1;
    }
    return gMotorNr;
}

int heima_sdk_get_motor_actuals(HeimaMotorActual* out_array, int out_count){
    if(out_array == nullptr || out_count <= 0){
        return -1;
    }
    std::lock_guard<std::mutex> lock(gMutex);
    if(!gInitialized){
        return -2;
    }
    if(out_count < gMotorNr){
        return -3;
    }
    std::vector<DriverSDK::motorActualStruct> actuals;
    actuals.resize((size_t)gMotorNr);
    int ret = DriverSDK::DriverSDK::instance().getMotorActual(actuals);
    if(ret != 0){
        return ret;
    }
    int i = 0;
    while(i < gMotorNr){
        out_array[i].pos = actuals[i].pos;
        out_array[i].vel = actuals[i].vel;
        out_array[i].tor = actuals[i].tor;
        out_array[i].temp = (int16_t)actuals[i].temp;
        out_array[i].status_word = (uint16_t)actuals[i].statusWord;
        out_array[i].error_code = (uint16_t)actuals[i].errorCode;
        out_array[i].reserved = 0;
        i++;
    }
    return gMotorNr;
}

int heima_sdk_get_imu(HeimaImu* out_imu){
    if(out_imu == nullptr){
        return -1;
    }
    std::lock_guard<std::mutex> lock(gMutex);
    if(!gInitialized){
        return -2;
    }
    DriverSDK::imuStruct imu{};
    DriverSDK::DriverSDK::instance().getIMU(imu);
    int i = 0;
    while(i < 3){
        out_imu->rpy[i] = imu.rpy[i];
        out_imu->gyr[i] = imu.gyr[i];
        out_imu->acc[i] = imu.acc[i];
        i++;
    }
    return 0;
}

char const* heima_sdk_version(void){
    std::lock_guard<std::mutex> lock(gMutex);
    if(!gInitialized){
        return "";
    }
    return gVersion.c_str();
}

} // extern "C"
