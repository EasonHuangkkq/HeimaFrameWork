//g++ -std=c++20 -O2 -pthread \
  mt_torque_demo.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp \
  -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
  -o build/mt_torque_demo
//sudo ./build/mt_torque_demo 1.0 10 config.xml 10 0.2
//sudo ./build/mt_torque_demo [currentA] [mode] [config.xml] [maxCurrentA] [sineHz]
#include "heima_driver_sdk.h"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}

bool allMotorsInOp(const std::vector<DriverSDK::motorActualStruct>& actuals) {
    for (const auto& act : actuals) {
        if ((act.statusWord & 0x007f) != 0x0037) {
            return false;
        }
    }
    return true;
}
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: mt_torque_demo [currentA] [mode] [config.xml] [maxCurrentA] [sineHz]\n";
        std::cout << "  currentA     target current amplitude in A (MT_Device + CST)\n";
        std::cout << "  mode         CiA-402 mode (default 10=CST)\n";
        std::cout << "  config.xml   config path (default config.xml)\n";
        std::cout << "  maxCurrentA  current limit in A (default 10)\n";
        std::cout << "  sineHz       if >0, modulate current with sin(2*pi*f*t)\n";
        return 0;
    }

    float targetCurrentA = 0.5f;
    if (argc > 1) {
        targetCurrentA = std::stof(argv[1]);
    }
    char mode = 10;
    if (argc > 2) {
        mode = static_cast<char>(std::stoi(argv[2]));
    }
    std::string configPath = "config.xml";
    if (argc > 3) {
        configPath = argv[3];
    }
    float maxCurrentA = 10.0f;
    if (argc > 4) {
        maxCurrentA = std::stof(argv[4]);
    }
    float sineHz = 0.0f;
    if (argc > 5) {
        sineHz = std::stof(argv[5]);
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    if (motorCount < 1) {
        std::cerr << "No motors configured; check config.xml aliases.\n";
        return 1;
    }

    sdk.setMode(std::vector<char>(motorCount, mode));

    const unsigned short maxCurrentRaw =
        static_cast<unsigned short>(std::lround(maxCurrentA * 100.0f));
    sdk.setMaxCurr(std::vector<unsigned short>(motorCount, maxCurrentRaw));

    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = 0.0f;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].enabled = 1;
    }

    const int maxWaitTicks = 10000; // ~10s at 1 kHz
    int waitTicks = 0;
    while (!g_stop) {
        sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);
        if ((waitTicks % 1000) == 0) {
            for (int i = 0; i < motorCount; ++i) {
                std::cout << "[wait " << waitTicks << " m" << i << "] status=0x"
                          << std::hex << actuals[i].statusWord
                          << " (state bits=0x" << (actuals[i].statusWord & 0x007f) << ")"
                          << std::dec << std::endl;
            }
        }
        if (allMotorsInOp(actuals)) {
            break;
        }
        if (++waitTicks > maxWaitTicks) {
            std::cerr << "Timeout waiting for all motors to reach OP.\n";
            break;
        }
        usleep(1000);
    }

    const float kTwoPi = 6.283185307179586f;
    auto t0 = std::chrono::steady_clock::now();
    int tick = 0;
    while (!g_stop) {
        float cmd = targetCurrentA;
        if (sineHz > 0.0f) {
            auto now = std::chrono::steady_clock::now();
            float t = std::chrono::duration<float>(now - t0).count();
            cmd = targetCurrentA * std::sin(kTwoPi * sineHz * t);
        }
        for (int i = 0; i < motorCount; ++i) {
            float dir = (i % 2 == 0) ? 1.0f : -1.0f;
            targets[i].tor = cmd * dir; // A for MT_Device + CST
            targets[i].enabled = 1;
        }
        sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);

        if ((tick++ % 1000) == 0) {
            for (int i = 0; i < motorCount; ++i) {
                std::cout << "[tick " << tick << " m" << i << "] pos=" << actuals[i].pos
                          << " vel=" << actuals[i].vel
                          << " tor=" << actuals[i].tor
                          << " status=0x" << std::hex << actuals[i].statusWord
                          << " (state bits=0x" << (actuals[i].statusWord & 0x007f) << ")"
                          << " error=0x" << actuals[i].errorCode
                          << " cmd_tor=" << std::dec << targets[i].tor
                          << " mode=" << static_cast<int>(mode)
                          << std::endl;
            }
        }
        usleep(1000);
    }

    for (int i = 0; i < motorCount; ++i) {
        targets[i].tor = 0.0f;
        targets[i].vel = 0.0f;
        targets[i].pos = 0.0f;
        targets[i].enabled = 0;
    }
    for (int i = 0; i < 200; ++i) {
        sdk.setMotorTarget(targets);
        usleep(1000);
    }
    return 0;
}
