// Build (Linux, requires EtherCAT library + headers in /usr/local):
// g++ -std=c++20 -O2 -pthread \
//   arm_mirror_demo.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp \
//   -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
//   -o build/arm_mirror_demo
//
// Run:
// sudo ./build/arm_mirror_demo --config config.xml --enable-right
//
// Override by explicit alias lists (right->left):
// sudo ./build/arm_mirror_demo --config config.xml --right 1,2,3,4,5,6,7,8 --left 9,10,11,12,13,14,15,16 \
//   --master right --enable-left
//
// Notes:
// - This demo mirrors left arm joint positions (limb=2) to right arm (limb=3).
// - It uses DriverSDK::getMotorActual()/setMotorTarget(); the SDK internally handles triple-buffered PDO pages.
// - For safety, right arm is DISABLED by default; pass --enable-right to actually command it.
//g++ -std=c++20 -O2 -pthread arm_mirror_demo.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm -o build/arm_mirror_demo
//sudo ./build/arm_mirror_demo --config config.xml --right 1,2,3,4,5,6,7,8 --left 9,10,11,12,13,14,15,16 --master right --calibrate-only
#include "heima_driver_sdk.h"

#include "config_xml.h"

#include <csignal>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;

void handleSignal(int) {
    g_stop = 1;
}

std::vector<float> parseCsvFloats(const std::string& s) {
    std::vector<float> out;
    size_t start = 0;
    while (start < s.size()) {
        size_t end = s.find(',', start);
        std::string token = (end == std::string::npos) ? s.substr(start) : s.substr(start, end - start);
        if (!token.empty()) {
            out.push_back(std::stof(token));
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

std::vector<int> parseCsvInts(const std::string& s) {
    std::vector<int> out;
    size_t start = 0;
    while (start < s.size()) {
        size_t end = s.find(',', start);
        std::string token = (end == std::string::npos) ? s.substr(start) : s.substr(start, end - start);
        if (!token.empty()) {
            out.push_back(std::stoi(token));
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--config config.xml] [--mode 8] [--period-us 1000] [--alpha 1.0]\n"
        << "              [--enable-right] [--enable-left] [--sign s0,s1,...] [--offset o0,o1,...]\n"
        << "              [--right a1,a2,...] [--left b1,b2,...] [--master left|right]\n"
        << "              [--calibrate] [--calibrate-only]\n"
        << "\n"
        << "  --config        Path to config.xml (default: config.xml)\n"
        << "  --mode          CiA-402 mode (default: 8)\n"
        << "  --period-us     Loop period in microseconds (default: 1000)\n"
        << "  --alpha         Follow smoothing [0..1], 1=direct, 0=freeze (default: 1)\n"
        << "  --enable-right  Enable right arm drives (default: disabled)\n"
        << "  --enable-left   Enable left arm drives (default: disabled)\n"
        << "  --sign          Per-joint sign for mirroring (len=dofArm, default all +1)\n"
        << "  --offset        Per-joint offset in rad (len=dofArm, default all 0)\n"
        << "  --right         Override right arm aliases (1-based)\n"
        << "  --left          Override left arm aliases (1-based)\n"
        << "  --master        Which side is master (default: left)\n"
        << "  --calibrate     Run SDO-based CountBias calibration on selected motors\n"
        << "  --calibrate-only  Calibrate then exit without running mirror loop\n";
}
}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    int modeInt = 8;
    int periodUs = 1000;
    float alpha = 1.0f;
    bool enableRight = false;
    bool enableLeft = false;
    std::vector<float> sign;
    std::vector<float> offset;
    std::string rightCsv;
    std::string leftCsv;
    std::string masterSide = "left";
    bool calibrate = false;
    bool calibrateOnly = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
            continue;
        }
        if (arg == "--mode" && i + 1 < argc) {
            modeInt = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--period-us" && i + 1 < argc) {
            periodUs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--alpha" && i + 1 < argc) {
            alpha = std::stof(argv[++i]);
            continue;
        }
        if (arg == "--enable-right") {
            enableRight = true;
            continue;
        }
        if (arg == "--enable-left") {
            enableLeft = true;
            continue;
        }
        if (arg == "--sign" && i + 1 < argc) {
            sign = parseCsvFloats(argv[++i]);
            continue;
        }
        if (arg == "--offset" && i + 1 < argc) {
            offset = parseCsvFloats(argv[++i]);
            continue;
        }
        if (arg == "--right" && i + 1 < argc) {
            rightCsv = argv[++i];
            continue;
        }
        if (arg == "--left" && i + 1 < argc) {
            leftCsv = argv[++i];
            continue;
        }
        if (arg == "--master" && i + 1 < argc) {
            masterSide = argv[++i];
            continue;
        }
        if (arg == "--calibrate") {
            calibrate = true;
            continue;
        }
        if (arg == "--calibrate-only") {
            calibrateOnly = true;
            calibrate = true;
            continue;
        }
        // Convenience: allow positional config.xml
        if (!arg.empty() && arg[0] != '-') {
            configPath = arg;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << "\n";
        printUsage(argv[0]);
        return 1;
    }

    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (periodUs <= 0) {
        periodUs = 1000;
    }
    if (masterSide != "left" && masterSide != "right") {
        std::cerr << "--master must be left or right\n";
        return 1;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    if (motorCount <= 0) {
        std::cerr << "No motors configured. Check <Motors> in config.xml.\n";
        return 1;
    }
    sdk.setMode(std::vector<char>(motorCount, static_cast<char>(modeInt)));

    std::vector<int> leftArmAlias;
    std::vector<int> rightArmAlias;
    if (!leftCsv.empty() || !rightCsv.empty()) {
        if (leftCsv.empty() || rightCsv.empty()) {
            std::cerr << "Both --left and --right must be provided when overriding aliases.\n";
            return 1;
        }
        leftArmAlias = parseCsvInts(leftCsv);
        rightArmAlias = parseCsvInts(rightCsv);
    } else {
        // Parse limb->alias mapping from config.xml to locate arms:
        DriverSDK::ConfigXML cfg(configPath.c_str());
        const std::vector<std::vector<int>> motorAlias = cfg.motorAlias(); // 0:LL,1:RL,2:LA,3:RA,4:waist,5:neck
        if (motorAlias.size() < 4) {
            std::cerr << "Invalid config.xml: missing limb mappings.\n";
            return 1;
        }
        leftArmAlias = motorAlias[2];
        rightArmAlias = motorAlias[3];
    }
    if (leftArmAlias.empty() || rightArmAlias.empty()) {
        std::cerr << "No arm motors found. Expect <Motor limb=\"2\"> and <Motor limb=\"3\"> entries.\n";
        return 1;
    }
    if (leftArmAlias.size() != rightArmAlias.size()) {
        std::cerr << "Left/right arm DOF mismatch: left=" << leftArmAlias.size()
                  << " right=" << rightArmAlias.size() << "\n";
        return 1;
    }

    const int dofArm = static_cast<int>(leftArmAlias.size());
    if (!sign.empty() && static_cast<int>(sign.size()) != dofArm) {
        std::cerr << "--sign length must equal dofArm=" << dofArm << "\n";
        return 1;
    }
    if (!offset.empty() && static_cast<int>(offset.size()) != dofArm) {
        std::cerr << "--offset length must equal dofArm=" << dofArm << "\n";
        return 1;
    }
    if (sign.empty()) {
        sign.assign(dofArm, 1.0f);
    }
    if (offset.empty()) {
        offset.assign(dofArm, 0.0f);
    }

    std::vector<int> leftIdx(dofArm), rightIdx(dofArm);
    for (int j = 0; j < dofArm; ++j) {
        const int la = leftArmAlias[j];
        const int ra = rightArmAlias[j];
        if (la <= 0 || la > motorCount || ra <= 0 || ra > motorCount) {
            std::cerr << "Arm alias out of range at joint " << j << ": left=" << la << " right=" << ra
                      << " motorCount=" << motorCount << "\n";
            return 1;
        }
        leftIdx[j] = la - 1;
        rightIdx[j] = ra - 1;
    }

    std::cout << "Config: " << configPath << "\n";
    std::cout << "Arm DOF: " << dofArm << " (master=" << masterSide << ")\n";
    std::cout << "Mode: " << modeInt << " period_us: " << periodUs << " alpha: " << alpha << "\n";
    std::cout << "Enable left: " << (enableLeft ? "yes" : "no") << " | enable right: " << (enableRight ? "yes" : "no")
              << "\n";
    for (int j = 0; j < dofArm; ++j) {
        std::cout << "  j" << j << ": left alias=" << leftArmAlias[j] << " (idx " << leftIdx[j] << ")"
                  << " -> right alias=" << rightArmAlias[j] << " (idx " << rightIdx[j] << ")"
                  << " sign=" << sign[j] << " offset=" << offset[j] << "\n";
    }

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    std::vector<float> slaveCmdPos(dofArm, 0.0f);

    const bool masterIsLeft = (masterSide == "left");
    const bool enableMaster = masterIsLeft ? enableLeft : enableRight;
    const bool enableSlave = masterIsLeft ? enableRight : enableLeft;
    const std::vector<int>& masterIdx = masterIsLeft ? leftIdx : rightIdx;
    const std::vector<int>& slaveIdx = masterIsLeft ? rightIdx : leftIdx;

    if (calibrate) {
        std::cout << "Calibrate: place arms at zero pose and keep still.\n";
        const int maxWaitTicks = static_cast<int>(50.0 * 1e6 / static_cast<double>(periodUs));
        int waitTick = 0;
        bool allOp = false;
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            for (int i = 0; i < motorCount; ++i) {
                targets[i].pos = actuals[i].pos;
                targets[i].vel = 0.0f;
                targets[i].tor = 0.0f;
                targets[i].kp = 0.0f;
                targets[i].kd = 0.0f;
                targets[i].enabled = 0;
            }
            for (int j = 0; j < dofArm; ++j) {
                targets[leftIdx[j]].enabled = 1;
                targets[rightIdx[j]].enabled = 1;
            }
            sdk.setMotorTarget(targets);

            allOp = true;
            for (int j = 0; j < dofArm; ++j) {
                if ((actuals[leftIdx[j]].statusWord & 0x007f) != 0x0037) {
                    allOp = false;
                }
                if ((actuals[rightIdx[j]].statusWord & 0x007f) != 0x0037) {
                    allOp = false;
                }
            }
            if (allOp) {
                break;
            }
            if (++waitTick > maxWaitTicks) {
                std::cerr << "Timeout waiting for OP (0x0037); SDO may fail.\n";
                break;
            }
            usleep(static_cast<useconds_t>(periodUs));
        }
        for (int k = 0; k < 10; ++k) {
            sdk.getMotorActual(actuals);
            usleep(static_cast<useconds_t>(periodUs));
        }
        auto calibrateAlias = [&](int alias) {
            const int idx = alias - 1;
            const int ret = sdk.calibrate(idx);
            if (ret == std::numeric_limits<int>::max()) {
                std::cout << "  alias " << alias << ": skipped (non-ECAT or not master 0)\n";
            } else if (ret == std::numeric_limits<int>::min()) {
                std::cout << "  alias " << alias << ": failed (SDO)\n";
            } else {
                std::cout << "  alias " << alias << ": CountBias=" << ret << "\n";
            }
        };
        std::cout << "Calibrating right arm aliases...\n";
        for (int a : rightArmAlias) {
            if (a > 0) {
                calibrateAlias(a);
            }
        }
        std::cout << "Calibrating left arm aliases...\n";
        for (int a : leftArmAlias) {
            if (a > 0) {
                calibrateAlias(a);
            }
        }
        if (calibrateOnly) {
            return 0;
        }
    }

    // Prime with current readings so we don't jump on first enable.
    sdk.getMotorActual(actuals);
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = actuals[i].pos;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].kp = 0.0f;
        targets[i].kd = 0.0f;
        targets[i].enabled = 0;
    }
    for (int j = 0; j < dofArm; ++j) {
        slaveCmdPos[j] = actuals[slaveIdx[j]].pos;
    }

    int tick = 0;
    while (!g_stop) {
        sdk.getMotorActual(actuals);

        // Default: hold everything at current position with zero vel/tor and disabled.
        for (int i = 0; i < motorCount; ++i) {
            targets[i].pos = actuals[i].pos;
            targets[i].vel = 0.0f;
            targets[i].tor = 0.0f;
            targets[i].kp = 0.0f;
            targets[i].kd = 0.0f;
            targets[i].enabled = 0;
        }

        // Master arm: optionally enable; always follow its own current pose.
        for (int j = 0; j < dofArm; ++j) {
            const int mi = masterIdx[j];
            targets[mi].pos = actuals[mi].pos;
            targets[mi].enabled = enableMaster ? 1 : 0;
        }

        // Slave arm: follow master arm pose (with sign/offset).
        for (int j = 0; j < dofArm; ++j) {
            const int mi = masterIdx[j];
            const int si = slaveIdx[j];
            const float desired = sign[j] * actuals[mi].pos + offset[j];
            slaveCmdPos[j] = slaveCmdPos[j] + alpha * (desired - slaveCmdPos[j]);
            targets[si].pos = slaveCmdPos[j];
            targets[si].enabled = enableSlave ? 1 : 0;
        }

        sdk.setMotorTarget(targets);

        if ((tick++ % 500) == 0) {
            const int j0 = 0;
            const int mi = masterIdx[j0];
            const int si = slaveIdx[j0];
            std::cout << "[tick " << tick << "]"
                      << " M0 pos=" << actuals[mi].pos << " sw=0x" << std::hex << actuals[mi].statusWord << std::dec
                      << " | S0 cmd=" << targets[si].pos << " act=" << actuals[si].pos << " sw=0x" << std::hex
                      << actuals[si].statusWord << std::dec << "\n";
        }

        usleep(static_cast<useconds_t>(periodUs));
    }

    // Disable everything on exit.
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = actuals[i].pos;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].kp = 0.0f;
        targets[i].kd = 0.0f;
        targets[i].enabled = 0;
    }
    for (int i = 0; i < 200; ++i) {
        sdk.setMotorTarget(targets);
        usleep(static_cast<useconds_t>(periodUs));
    }
    return 0;
}
