// Build (run in heimaSDK/):
// g++ -std=c++17 -O2 -pthread -DENABLE_RS232 test_ankle_hardware.cpp heima_driver_sdk.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp rs232.cpp yesense_imu.cpp yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp rs485.cpp can.cpp -I. -I../loong_utility-main/third_party/include/eigen3 -I../heima_simulator/code/ankle_solver/cpp /Users/shenghuang/Downloads/heima/heimaFrameWork/heima_simulator/code/ankle_solver/cpp/ankle_solver.cpp -I/usr/local/include -L/usr/local/lib -lethercat -lm -o build/test_ankle_hardware

#include "heima_driver_sdk.h"
#include "ankle_solver.h"

#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <vector>

namespace {
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) { g_stop = 1; }

float stepToward(float current, float target, float maxStep) {
    if (maxStep <= 0.0f) return current;
    const float err = target - current;
    if (std::fabs(err) <= maxStep) return target;
    return current + ((err > 0.0f) ? maxStep : -maxStep);
}

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [--config config.xml] [--pitch 0.0] [--roll 0.0]\n";
}

std::string getErrorString(unsigned short errorCode) {
    if (errorCode == 0) return "OK";
    std::string errs = "";
    if (errorCode & 0x0002) errs += "[堵转] ";
    if (errorCode & 0x0004) errs += "[低压] ";
    if (errorCode & 0x0008) errs += "[过压] ";
    if (errorCode & 0x0010) errs += "[相线过流] ";
    if (errorCode & 0x0080) errs += "[标定错误] ";
    if (errorCode & 0x0100) errs += "[超速] ";
    if (errorCode & 0x0800) errs += "[驱动器过温] ";
    if (errorCode & 0x1000) errs += "[电机过温] ";
    if (errorCode & 0x2000) errs += "[编码器未校准] ";
    if (errorCode & 0x4000) errs += "[编码器数据错] ";
    if (errs.empty()) errs = "[未知异常] ";
    return errs;
}

}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    double pitch = 0.0;
    double roll = 0.0;
    
    // 脚踝电机的 Alias (左脚 5,6; 右脚 11,12)
    std::vector<int> aliases_f = {5, 11};
    std::vector<int> aliases_e = {6, 12};
    std::vector<int> indices_f;
    std::vector<int> indices_e;

    int periodUs = 1000;
    double maxVel = 0.2; // rad/s

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]); return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--pitch" && i + 1 < argc) {
            pitch = std::stod(argv[++i]);
        } else if (arg == "--roll" && i + 1 < argc) {
            roll = std::stod(argv[++i]);
        } else if (arg == "--max-vel" && i + 1 < argc) {
            maxVel = std::stod(argv[++i]);
        }
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    AnkleSolver solver;
    auto [target_theta_f, target_theta_e] = solver.solve(pitch, roll);
    std::cout << "Ankle target Pitch=" << pitch << ", Roll=" << roll << "\n";
    std::cout << "Computed motor targets: Theta_F=" << target_theta_f << " rad, Theta_E=" << target_theta_e << " rad\n";

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    
    for (int alias : aliases_f) {
        if (alias - 1 < motorCount) indices_f.push_back(alias - 1);
    }
    for (int alias : aliases_e) {
        if (alias - 1 < motorCount) indices_e.push_back(alias - 1);
    }

    if (indices_f.empty() || indices_e.empty()) {
        std::cerr << "Motor aliases out of bounds! motorCount=" << motorCount << "\n";
        return 1;
    }

    // ============================================
    // 改回 Mode 5 (PVT 模式)！
    // 并且根据要求下发 kp=12.0, kd=0.2 的刚度参数
    // ============================================
    std::vector<char> modes(motorCount, 5); 
    sdk.setMode(modes);

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);

    std::cout << "Waiting for motors to reach OP...\n";
    while (!g_stop) {
        sdk.getMotorActual(actuals);
        bool allOp = true;
        for (int i = 0; i < motorCount; ++i) {
            if ((actuals[i].statusWord & 0x007f) != 0x0037) { allOp = false; break; }
        }
        if (allOp) break;

        for(int i = 0; i < motorCount; i++) {
            targets[i].pos = actuals[i].pos;
            targets[i].enabled = 1;
            targets[i].kp = 12.0f;
            targets[i].kd = 0.2f;
            targets[i].tor = 0.0f;
        }
        sdk.setMotorTarget(targets);
        usleep(periodUs);
    }

    if (g_stop) return 0;
    std::cout << "All motors OP! Starting smooth motion to ankle targets...\n";

    std::vector<float> currents_f(indices_f.size());
    std::vector<float> currents_e(indices_e.size());
    for(size_t i = 0; i < indices_f.size(); ++i) currents_f[i] = actuals[indices_f[i]].pos;
    for(size_t i = 0; i < indices_e.size(); ++i) currents_e[i] = actuals[indices_e[i]].pos;
    
    float maxStep = maxVel * (periodUs * 1e-6);
    int printTick = 0;

    while (!g_stop) {
        sdk.getMotorActual(actuals);

        for(int i = 0; i < motorCount; i++) {
            targets[i].pos = actuals[i].pos; // hold others
            targets[i].vel = 0;
            targets[i].tor = 0;
            targets[i].kp = 12.0f;
            targets[i].kd = 0.2f;
            targets[i].enabled = 1;
        }

        for(size_t i = 0; i < indices_f.size(); ++i) {
            currents_f[i] = stepToward(currents_f[i], target_theta_f, maxStep);
            targets[indices_f[i]].pos = currents_f[i];
        }
        for(size_t i = 0; i < indices_e.size(); ++i) {
            currents_e[i] = stepToward(currents_e[i], target_theta_e, maxStep);
            targets[indices_e[i]].pos = currents_e[i];
        }

        sdk.setMotorTarget(targets);

        if (++printTick % 100 == 0) {
            std::cout << "Left Ankle (5,6) -> \n"
                      << "  F: Cmd=" << targets[indices_f[0]].pos << " Act=" << actuals[indices_f[0]].pos << " Err=" << getErrorString(actuals[indices_f[0]].errorCode) << "\n"
                      << "  E: Cmd=" << targets[indices_e[0]].pos << " Act=" << actuals[indices_e[0]].pos << " Err=" << getErrorString(actuals[indices_e[0]].errorCode) << "\n";
            if(indices_f.size() > 1 && indices_e.size() > 1) {
                std::cout << "Right Ankle (11,12) -> \n"
                          << "  F: Cmd=" << targets[indices_f[1]].pos << " Act=" << actuals[indices_f[1]].pos << " Err=" << getErrorString(actuals[indices_f[1]].errorCode) << "\n"
                          << "  E: Cmd=" << targets[indices_e[1]].pos << " Act=" << actuals[indices_e[1]].pos << " Err=" << getErrorString(actuals[indices_e[1]].errorCode) << "\n";
            }
        }

        usleep(periodUs);
    }

    std::cout << "Disabling motors before exit...\n";
    for(int j=0; j<200; j++) {
        sdk.getMotorActual(actuals);
        for(int i = 0; i < motorCount; i++) targets[i].enabled = 0;
        sdk.setMotorTarget(targets);
        usleep(periodUs);
    }

    return 0;
}
