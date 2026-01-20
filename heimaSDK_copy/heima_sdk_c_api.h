/*
 * C ABI wrapper for DriverSDK (for ctypes / FFI).
 *
 * Keep this header C-compatible: do not include any C++ headers here.
 */

#pragma once

#include <stdint.h>

#if defined(_WIN32) || defined(_WIN64)
#define HEIMA_SDK_API __declspec(dllexport)
#else
#define HEIMA_SDK_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HeimaImu{
    float rpy[3];
    float gyr[3];
    float acc[3];
} HeimaImu;

// Stable layout for ctypes:
// sizeof(HeimaMotorActual) == 20 on common ABIs (4-byte alignment).
typedef struct HeimaMotorActual{
    float pos;
    float vel;
    float tor;
    int16_t temp;
    uint16_t status_word;
    uint16_t error_code;
    uint16_t reserved;
} HeimaMotorActual;

HEIMA_SDK_API int heima_sdk_init(char const* config_xml_path);
HEIMA_SDK_API int heima_sdk_get_total_motor_nr(void);
HEIMA_SDK_API int heima_sdk_get_motor_actuals(HeimaMotorActual* out_array, int out_count);
HEIMA_SDK_API int heima_sdk_get_imu(HeimaImu* out_imu);
HEIMA_SDK_API char const* heima_sdk_version(void);

#ifdef __cplusplus
}
#endif

