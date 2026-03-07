/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#include "heima_driver_sdk.h"
#include <csignal>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}
}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    if (argc > 1) {
        configPath = argv[1];
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    DriverSDK::imuStruct imu{};
    int tick = 0;
    while (!g_stop) {
        sdk.getIMU(imu);
        if ((tick++ % 100) == 0) {
            std::cout << "rpy: " << imu.rpy[0] << " " << imu.rpy[1] << " " << imu.rpy[2]
                      << " gyr: " << imu.gyr[0] << " " << imu.gyr[1] << " " << imu.gyr[2]
                      << " acc: " << imu.acc[0] << " " << imu.acc[1] << " " << imu.acc[2]
                      << std::endl;
        }
        usleep(10000); // 100 Hz
    }
    return 0;
}
