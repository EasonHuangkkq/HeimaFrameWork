#include "heima_driver_sdk.h"
#include <iostream>
#include <vector>
#include <string>
#include <csignal>
#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}
}

int main(int argc, char** argv) {
    // Note: class name equals namespace; use fully qualified form.
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();

    // Optional CLI:
    // - For MT_Device in CST (mode=10), argv[1]=target current (A), where 1.0A => raw 100.
    // - For other modes/devices, argv[1] keeps the original meaning in this demo.
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

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    // Delay setMode until after init so we know motor count.
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    // If multiple motors are present, replicate mode for all, then set.
    std::vector<char> modes(motorCount, mode);
    sdk.setMode(modes);
    if (motorCount < 1) {
        std::cerr << "No motors configured; ensure config.xml has alias=1 set.\n";
        return 1;
    }

    // Apply the same target to all motors unless you want per-motor tuning.
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    for (int i = 0; i < motorCount; ++i) {
        targets[i].tor = 0.0f;            // hold zero torque while driving to OP
        targets[i].vel = 0;
        targets[i].pos = 0;
        targets[i].enabled = 1;           // enable and drive CiA-402 state machine
    }

    // Wait until all motors reach OP (0x37) before applying non-zero targets.
    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    const int maxWaitTicks = 10000; // ~10s at 1kHz
    int waitTicks = 0;
    while (!g_stop) {
        sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);
        bool allOp = true;
        for (int i = 0; i < motorCount; ++i) {
            if ((actuals[i].statusWord & 0x007f) != 0x0037) {
                allOp = false;
            }
        }
        if ((waitTicks % 1000) == 0) {
            for (int i = 0; i < motorCount; ++i) {
                std::cout << "[wait " << waitTicks << " m" << i << "] status=0x"
                          << std::hex << actuals[i].statusWord
                          << " (state bits=0x" << (actuals[i].statusWord & 0x007f) << ")"
                          << std::dec << std::endl;
            }
        }
        if (allOp) {
            break;
        }
        if (++waitTicks > maxWaitTicks) {
            std::cerr << "Timeout waiting for all motors to reach OP." << std::endl;
            break;
        }
        usleep(1000);
    }

    int tick = 0;
    while (!g_stop) {
        for (int i = 0; i < motorCount; ++i) {
            targets[i].tor = targetCurrentA;  // A for MT_Device+CST (mode=10)
        }
        sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);

        if ((tick++ % 1000) == 0) {
            for (int i = 0; i < motorCount; ++i) {
                unsigned short sw = actuals[i].statusWord;
                std::cout << "[tick " << tick << " m" << i << "] pos=" << actuals[i].pos
                          << " vel=" << actuals[i].vel
                          << " tor=" << actuals[i].tor
                          << " status=0x" << std::hex << sw
                          << " (state bits=0x" << (sw & 0x007f) << ")"
                          << " error=0x" << actuals[i].errorCode
                          << " cmd_tor=" << std::dec << targets[i].tor
                          << " mode=" << static_cast<int>(mode)
                          << std::endl;
            }
        }
        usleep(1000); // 1 kHz loop
    }

    // Graceful stop: disable all drives and send a few cycles to apply.
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
