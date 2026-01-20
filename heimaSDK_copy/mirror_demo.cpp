// Build (Linux, requires EtherCAT library + headers in /usr/local):
// g++ -std=c++20 -O2 -pthread \
//   mirror_demo.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp \
//   -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
//   -o build/mirror_demo
//
// Run:
// sudo ./build/mirror_demo --config config.xml --enable-right
//
// Override by explicit alias lists (right->left):
// sudo ./build/mirror_demo --config config.xml --right 1,2,3,4,5,6,7,8 --left 9,10,11,12,13,14,15,16 \
//   --master right --enable-left
//
// Notes:
// - This demo mirrors master-side arm+leg joint positions to the other side.
// - It uses DriverSDK::getMotorActual()/setMotorTarget(); the SDK internally handles triple-buffered PDO pages.
// - For safety, slave side is DISABLED by default; pass --enable-left/--enable-right to actually command a side.
// - Waist is fixed (hold current position) by default when legs are active; disable with --free-waist.
//g++ -std=c++20 -O2 -pthread mirror_demo.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm -o build/mirror_demo
//sudo ./build/mirror_demo --config config.xml --right 1,2,3,4,5,6,7,8 --left 9,10,11,12,13,14,15,16 --master right --calibrate-only
#include "heima_driver_sdk.h"

#include "config_xml.h"

#include <algorithm>
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
        << "              [--leg-sign s0,s1,...] [--leg-offset o0,o1,...]\n"
        << "              [--right a1,a2,...] [--left b1,b2,...] [--master left|right]\n"
        << "              [--arms-only] [--legs-only]\n"
        << "              [--relative]\n"
        << "              [--calibrate] [--calibrate-only]\n"
        << "              [--fix-waist] [--free-waist]\n"
        << "              [--left-leg a1,a2,...] [--right-leg b1,b2,...]\n"
        << "\n"
        << "  --config        Path to config.xml (default: config.xml)\n"
        << "  --mode          CiA-402 mode (default: 8)\n"
        << "  --period-us     Loop period in microseconds (default: ECAT period from config.xml, fallback 1000)\n"
        << "  --alpha         Follow smoothing [0..1], 1=direct, 0=freeze (default: 1; legs default to 0.02 if not set)\n"
        << "  --enable-right  Enable RIGHT side drives (default: disabled)\n"
        << "  --enable-left   Enable LEFT side drives (default: disabled)\n"
        << "  --sign          Per-joint sign for mirroring (len=dofArm, default all +1)\n"
        << "  --offset        Per-joint offset in rad (len=dofArm, default all 0)\n"
        << "  --leg-sign      Per-joint sign for leg mirroring (len=dofLeg, default all +1)\n"
        << "  --leg-offset    Per-joint offset for leg mirroring in rad (len=dofLeg, default all 0)\n"
        << "  --right         Override right arm aliases (1-based)\n"
        << "  --left          Override left arm aliases (1-based)\n"
        << "  --master        Which side is master (default: left)\n"
        << "  --arms-only     Run only arm mirroring (ignore legs)\n"
        << "  --legs-only     Run only leg mirroring (ignore arms)\n"
        << "  --relative      Mirror DELTA from start pose (no initial jump); recommended for legs\n"
        << "  --calibrate     Run SDO-based CountBias calibration on selected motors\n"
        << "  --calibrate-only  Calibrate then exit without running mirror loop\n"
        << "  --fix-waist     Fix waist position (hold current)\n"
        << "  --free-waist    Do not fix waist position (default when --arms-only)\n"
        << "  --left-leg      Override left leg aliases (1-based)\n"
        << "  --right-leg     Override right leg aliases (1-based)\n";
}
}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    int modeInt = 8;
    int periodUs = 1000;
    bool periodUsSet = false;
    float alpha = 1.0f;
    bool alphaSet = false;
    bool enableRight = false;
    bool enableLeft = false;
    std::vector<float> sign;
    std::vector<float> offset;
    std::vector<float> legSign;
    std::vector<float> legOffset;
    std::string rightCsv;
    std::string leftCsv;
    std::string masterSide = "left";
    bool armsOnly = false;
    bool legsOnly = false;
    bool relative = false;
    bool calibrate = false;
    bool calibrateOnly = false;
    bool fixWaist = false;
    bool fixWaistSet = false;
    std::string leftLegCsv;
    std::string rightLegCsv;

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
            periodUsSet = true;
            continue;
        }
        if (arg == "--alpha" && i + 1 < argc) {
            alpha = std::stof(argv[++i]);
            alphaSet = true;
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
        if (arg == "--leg-sign" && i + 1 < argc) {
            legSign = parseCsvFloats(argv[++i]);
            continue;
        }
        if (arg == "--leg-offset" && i + 1 < argc) {
            legOffset = parseCsvFloats(argv[++i]);
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
        if (arg == "--arms-only") {
            armsOnly = true;
            continue;
        }
        if (arg == "--legs-only") {
            legsOnly = true;
            continue;
        }
        if (arg == "--relative") {
            relative = true;
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
        if (arg == "--fix-waist") {
            fixWaist = true;
            fixWaistSet = true;
            continue;
        }
        if (arg == "--free-waist") {
            fixWaist = false;
            fixWaistSet = true;
            continue;
        }
        if (arg == "--left-leg" && i + 1 < argc) {
            leftLegCsv = argv[++i];
            continue;
        }
        if (arg == "--right-leg" && i + 1 < argc) {
            rightLegCsv = argv[++i];
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
    if (armsOnly && legsOnly) {
        std::cerr << "Cannot use --arms-only with --legs-only\n";
        return 1;
    }
    const bool runArms = !legsOnly;
    const bool runLegs = !armsOnly;
    if (!fixWaistSet) {
        fixWaist = runLegs;
    }
    const bool anyEnabled = enableLeft || enableRight;
    const bool enableWaist = fixWaist && anyEnabled;

    float armAlpha = alpha;
    float legAlpha = alpha;
    if (runLegs && !alphaSet) {
        legAlpha = 0.02f;
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
    std::cout << "[mirror_demo] safety: gate enable by valid StatusWord (PDO complete)\n";
    auto isValidSw = [](unsigned short sw) {
        return sw != 0 && sw != 0xffff;
    };
    
    std::vector<int> leftArmAlias;
    std::vector<int> rightArmAlias;
    std::vector<int> leftLegAlias;
    std::vector<int> rightLegAlias;
    std::vector<int> waistAlias;

    DriverSDK::ConfigXML cfg(configPath.c_str());
    if (!periodUsSet) {
        const int periodNs = cfg.attribute("ECAT", 0, "period");
        if (periodNs > 0) {
            periodUs = periodNs / 1000;
        }
        if (periodUs <= 0) {
            periodUs = 1000;
        }
    }
    const std::vector<std::vector<int>> motorAlias = cfg.motorAlias(); // 0:LL,1:RL,2:LA,3:RA,4:waist,5:neck
    if (motorAlias.size() < 6) {
        std::cerr << "Invalid config.xml: missing limb mappings.\n";
        return 1;
    }

    leftArmAlias = motorAlias[2];
    rightArmAlias = motorAlias[3];
    leftLegAlias = motorAlias[0];
    rightLegAlias = motorAlias[1];
    waistAlias = motorAlias[4];

    auto filterPositive = [](std::vector<int> const& in) {
        std::vector<int> out;
        out.reserve(in.size());
        for (int a : in) {
            if (a > 0) {
                out.push_back(a);
            }
        }
        return out;
    };

    if (!leftCsv.empty() || !rightCsv.empty()) {
        if (leftCsv.empty() || rightCsv.empty()) {
            std::cerr << "Both --left and --right must be provided when overriding arm aliases.\n";
            return 1;
        }
        leftArmAlias = parseCsvInts(leftCsv);
        rightArmAlias = parseCsvInts(rightCsv);
    }
    if (!leftLegCsv.empty() || !rightLegCsv.empty()) {
        if (leftLegCsv.empty() || rightLegCsv.empty()) {
            std::cerr << "Both --left-leg and --right-leg must be provided when overriding leg aliases.\n";
            return 1;
        }
        leftLegAlias = parseCsvInts(leftLegCsv);
        rightLegAlias = parseCsvInts(rightLegCsv);
    }

    leftArmAlias = filterPositive(leftArmAlias);
    rightArmAlias = filterPositive(rightArmAlias);
    leftLegAlias = filterPositive(leftLegAlias);
    rightLegAlias = filterPositive(rightLegAlias);
    waistAlias = filterPositive(waistAlias);

    if (!runArms) {
        if (!leftCsv.empty() || !rightCsv.empty() || !sign.empty() || !offset.empty()) {
            std::cout << "[WARN] --legs-only: ignoring arm args (--left/--right/--sign/--offset)\n";
        }
        leftArmAlias.clear();
        rightArmAlias.clear();
        sign.clear();
        offset.clear();
    }
    if (!runLegs) {
        if (!leftLegCsv.empty() || !rightLegCsv.empty() || !legSign.empty() || !legOffset.empty()) {
            std::cout << "[WARN] --arms-only: ignoring leg args (--left-leg/--right-leg/--leg-sign/--leg-offset)\n";
        }
        leftLegAlias.clear();
        rightLegAlias.clear();
        legSign.clear();
        legOffset.clear();
    }

    int dofArm = 0;
    if (runArms) {
        if (leftArmAlias.empty() || rightArmAlias.empty()) {
            std::cerr << "No arm motors found. Expect <Motor limb=\"2\"> and <Motor limb=\"3\"> entries.\n";
            return 1;
        }
        if (leftArmAlias.size() != rightArmAlias.size()) {
            std::cerr << "Left/right arm DOF mismatch: left=" << leftArmAlias.size()
                      << " right=" << rightArmAlias.size() << "\n";
            return 1;
        }
        dofArm = static_cast<int>(leftArmAlias.size());
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
    }

    int dofLeg = 0;
    if (runLegs) {
        if (leftLegAlias.empty() || rightLegAlias.empty()) {
            std::cerr << "No leg motors found. Expect <Motor limb=\"0\"> and <Motor limb=\"1\"> entries.\n";
            return 1;
        }
        if (leftLegAlias.size() != rightLegAlias.size()) {
            std::cerr << "Left/right leg DOF mismatch: left=" << leftLegAlias.size()
                      << " right=" << rightLegAlias.size() << "\n";
            return 1;
        }
        dofLeg = static_cast<int>(leftLegAlias.size());
        if (!legSign.empty() && static_cast<int>(legSign.size()) != dofLeg) {
            std::cerr << "--leg-sign length must equal dofLeg=" << dofLeg << "\n";
            return 1;
        }
        if (!legOffset.empty() && static_cast<int>(legOffset.size()) != dofLeg) {
            std::cerr << "--leg-offset length must equal dofLeg=" << dofLeg << "\n";
            return 1;
        }
        if (legSign.empty()) {
            legSign.assign(dofLeg, 1.0f);
        }
        if (legOffset.empty()) {
            legOffset.assign(dofLeg, 0.0f);
        }
    }

    const int dofWaist = static_cast<int>(waistAlias.size());

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
    if (runArms) {
        std::cout << "Arm DOF: " << dofArm << " (master=" << masterSide << ")\n";
    }
    if (runLegs) {
        std::cout << "Leg DOF: " << dofLeg << " (master=" << masterSide << ")\n";
    }
    if (dofWaist > 0) {
        std::cout << "Waist DOF: " << dofWaist << "\n";
    }
    std::cout << "Mode: " << modeInt << " period_us: " << periodUs;
    if (runArms) {
        std::cout << " arm_alpha: " << armAlpha;
    }
    if (runLegs) {
        std::cout << " leg_alpha: " << legAlpha;
        if (!alphaSet) {
            std::cout << " (default)";
        }
    }
    std::cout << "\n";
    std::cout << "Enable left: " << (enableLeft ? "yes" : "no") << " | enable right: " << (enableRight ? "yes" : "no")
              << "\n";
    if (dofWaist > 0) {
        std::cout << "Fix waist: " << (fixWaist ? "yes" : "no")
                  << " | waist enabled: " << (enableWaist ? "yes" : "no")
                  << " (use --free-waist to disable)\n";
    }
    std::cout << "Relative mirror: " << (relative ? "yes" : "no") << "\n";
    for (int j = 0; j < dofArm; ++j) {
        std::cout << "  arm j" << j << ": left alias=" << leftArmAlias[j] << " (idx " << leftIdx[j] << ")"
                  << " -> right alias=" << rightArmAlias[j] << " (idx " << rightIdx[j] << ")"
                  << " sign=" << sign[j] << " offset=" << offset[j] << "\n";
    }
    std::vector<int> leftLegIdx(dofLeg), rightLegIdx(dofLeg);
    if (dofLeg > 0) {
        for (int j = 0; j < dofLeg; ++j) {
            const int la = leftLegAlias[j];
            const int ra = rightLegAlias[j];
            if (la <= 0 || la > motorCount || ra <= 0 || ra > motorCount) {
                std::cerr << "Leg alias out of range at joint " << j << ": left=" << la << " right=" << ra
                          << " motorCount=" << motorCount << "\n";
                return 1;
            }
            leftLegIdx[j] = la - 1;
            rightLegIdx[j] = ra - 1;
            std::cout << "  leg j" << j << ": left alias=" << la << " (idx " << leftLegIdx[j] << ")"
                      << " -> right alias=" << ra << " (idx " << rightLegIdx[j] << ")"
                      << " sign=" << legSign[j] << " offset=" << legOffset[j] << "\n";
        }
    }

    std::vector<int> waistIdx(dofWaist);
    for (int j = 0; j < dofWaist; ++j) {
        const int a = waistAlias[j];
        if (a <= 0 || a > motorCount) {
            std::cerr << "Waist alias out of range at joint " << j << ": alias=" << a << " motorCount=" << motorCount
                      << "\n";
            return 1;
        }
        waistIdx[j] = a - 1;
    }

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    std::vector<float> slaveCmdPos(dofArm, 0.0f);
    std::vector<float> legSlaveCmdPos(dofLeg, 0.0f);
    std::vector<float> waistCmdPos(dofWaist, 0.0f);
    
    const bool masterIsLeft = (masterSide == "left");
    const bool enableMaster = masterIsLeft ? enableLeft : enableRight;
    const bool enableSlave = masterIsLeft ? enableRight : enableLeft;
    const std::vector<int>& masterIdx = masterIsLeft ? leftIdx : rightIdx;
    const std::vector<int>& slaveIdx = masterIsLeft ? rightIdx : leftIdx;
    const std::vector<int>& masterLegIdx = masterIsLeft ? leftLegIdx : rightLegIdx;
    const std::vector<int>& slaveLegIdx = masterIsLeft ? rightLegIdx : leftLegIdx;

    if (calibrate) {
        if (runArms && runLegs) {
            std::cout << "Calibrate: place arms+legs at zero pose and keep still.\n";
        } else if (runArms) {
            std::cout << "Calibrate: place arms at zero pose and keep still.\n";
        } else if (runLegs) {
            std::cout << "Calibrate: place legs at zero pose and keep still.\n";
        } else {
            std::cout << "Calibrate: nothing selected.\n";
        }
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
                const int li = leftIdx[j];
                const int ri = rightIdx[j];
                if (isValidSw(actuals[li].statusWord)) {
                    targets[li].enabled = 1;
                }
                if (isValidSw(actuals[ri].statusWord)) {
                    targets[ri].enabled = 1;
                }
            }
            for (int j = 0; j < dofLeg; ++j) {
                const int li = leftLegIdx[j];
                const int ri = rightLegIdx[j];
                if (isValidSw(actuals[li].statusWord)) {
                    targets[li].enabled = 1;
                }
                if (isValidSw(actuals[ri].statusWord)) {
                    targets[ri].enabled = 1;
                }
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
            for (int j = 0; j < dofLeg; ++j) {
                if ((actuals[leftLegIdx[j]].statusWord & 0x007f) != 0x0037) {
                    allOp = false;
                }
                if ((actuals[rightLegIdx[j]].statusWord & 0x007f) != 0x0037) {
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

        if (runArms) {
            std::cout << "Calibrating right arm aliases...\n";
            for (int a : rightArmAlias) {
                calibrateAlias(a);
            }
            std::cout << "Calibrating left arm aliases...\n";
            for (int a : leftArmAlias) {
                calibrateAlias(a);
            }
        }
        if (runLegs) {
            std::cout << "Calibrating right leg aliases...\n";
            for (int a : rightLegAlias) {
                calibrateAlias(a);
            }
            std::cout << "Calibrating left leg aliases...\n";
            for (int a : leftLegAlias) {
                calibrateAlias(a);
            }
        }

        if (calibrateOnly) {
            // Disable everything on exit.
            sdk.getMotorActual(actuals);
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
    }

    auto isOp = [](unsigned short sw) {
        return (sw & 0x007f) == 0x0037;
    };

    // Wait for OP for motors that we will actually enable, to avoid commanding while PDO/AL states are still settling.
    std::vector<int> enableIdx;
    enableIdx.reserve(dofArm * 2 + dofLeg * 2 + dofWaist);
    if (runArms) {
        if (enableMaster) {
            enableIdx.insert(enableIdx.end(), masterIdx.begin(), masterIdx.end());
        }
        if (enableSlave) {
            enableIdx.insert(enableIdx.end(), slaveIdx.begin(), slaveIdx.end());
        }
    }
    if (runLegs) {
        if (enableMaster) {
            enableIdx.insert(enableIdx.end(), masterLegIdx.begin(), masterLegIdx.end());
        }
        if (enableSlave) {
            enableIdx.insert(enableIdx.end(), slaveLegIdx.begin(), slaveLegIdx.end());
        }
    }
    if (enableWaist) {
        enableIdx.insert(enableIdx.end(), waistIdx.begin(), waistIdx.end());
    }
    std::sort(enableIdx.begin(), enableIdx.end());
    enableIdx.erase(std::unique(enableIdx.begin(), enableIdx.end()), enableIdx.end());
    if (!enableIdx.empty()) {
        std::cout << "[init] waiting OP for " << enableIdx.size() << " motor(s)...\n";
        const int maxWaitTicks = static_cast<int>(10.0 * 1e6 / static_cast<double>(periodUs));
        int waitTick = 0;
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
            for (int idx : enableIdx) {
                if (isValidSw(actuals[idx].statusWord)) {
                    targets[idx].enabled = 1;
                }
            }
            sdk.setMotorTarget(targets);

            bool allOp = true;
            int firstNotOp = -1;
            for (int idx : enableIdx) {
                if (!isOp(actuals[idx].statusWord)) {
                    allOp = false;
                    firstNotOp = idx;
                    break;
                }
            }
            if (allOp) {
                break;
            }
            if ((waitTick % 500) == 0 && firstNotOp >= 0) {
                std::cout << "[init] waiting OP... first_not_op idx=" << firstNotOp
                          << " sw=0x" << std::hex << actuals[firstNotOp].statusWord << std::dec << "\n";
            }
            if (++waitTick > maxWaitTicks) {
                std::cerr << "[WARN] timeout waiting OP; continue anyway\n";
                break;
            }
            usleep(static_cast<useconds_t>(periodUs));
        }
    }

    // Wait for valid feedback on the master/reference side (even if it stays DISABLED).
    std::vector<int> refIdx;
    refIdx.reserve(dofArm + dofLeg);
    if (runArms) {
        refIdx.insert(refIdx.end(), masterIdx.begin(), masterIdx.end());
    }
    if (runLegs) {
        refIdx.insert(refIdx.end(), masterLegIdx.begin(), masterLegIdx.end());
    }
    std::sort(refIdx.begin(), refIdx.end());
    refIdx.erase(std::unique(refIdx.begin(), refIdx.end()), refIdx.end());
    if (!refIdx.empty()) {
        std::cout << "[init] waiting master feedback valid...\n";
        const int maxWaitTicks = static_cast<int>(5.0 * 1e6 / static_cast<double>(periodUs));
        int waitTick = 0;
        while (!g_stop) {
            const int ret = sdk.getMotorActual(actuals);
            for (int i = 0; i < motorCount; ++i) {
                targets[i].pos = actuals[i].pos;
                targets[i].vel = 0.0f;
                targets[i].tor = 0.0f;
                targets[i].kp = 0.0f;
                targets[i].kd = 0.0f;
                targets[i].enabled = 0;
            }
            for (int idx : enableIdx) {
                if (isValidSw(actuals[idx].statusWord)) {
                    targets[idx].enabled = 1;
                }
            }
            sdk.setMotorTarget(targets);
            bool ok = (ret == 0);
            for (int idx : refIdx) {
                if (!isValidSw(actuals[idx].statusWord)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                break;
            }
            if ((waitTick % 500) == 0) {
                std::cout << "[init] ecat_stalled=" << ((ret < 0) ? -ret : 0) << "\n";
            }
            if (++waitTick > maxWaitTicks) {
                std::cerr << "[WARN] timeout waiting master feedback; continue anyway\n";
                break;
            }
            usleep(static_cast<useconds_t>(periodUs));
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
    
    // Initialize leg command positions (so we don't jump on first enable).
    if (dofLeg > 0) {
        for (int j = 0; j < dofLeg; ++j) {
            legSlaveCmdPos[j] = actuals[slaveLegIdx[j]].pos;
        }
    }

    // If using relative mirror, capture the start pose for master & slave to avoid initial jump.
    std::vector<float> armMaster0(dofArm, 0.0f), armSlave0(dofArm, 0.0f);
    std::vector<float> legMaster0(dofLeg, 0.0f), legSlave0(dofLeg, 0.0f);
    std::vector<char> arm0Valid(dofArm, 0), leg0Valid(dofLeg, 0);
    if (relative) {
        for (int j = 0; j < dofArm; ++j) {
            const int mi = masterIdx[j];
            const int si = slaveIdx[j];
            if (isValidSw(actuals[mi].statusWord) && isValidSw(actuals[si].statusWord)) {
                armMaster0[j] = actuals[mi].pos;
                armSlave0[j] = actuals[si].pos;
                arm0Valid[j] = 1;
                slaveCmdPos[j] = actuals[si].pos;
            }
        }
        for (int j = 0; j < dofLeg; ++j) {
            const int mi = masterLegIdx[j];
            const int si = slaveLegIdx[j];
            if (isValidSw(actuals[mi].statusWord) && isValidSw(actuals[si].statusWord)) {
                legMaster0[j] = actuals[mi].pos;
                legSlave0[j] = actuals[si].pos;
                leg0Valid[j] = 1;
                legSlaveCmdPos[j] = actuals[si].pos;
            }
        }
    }
    
    // Initialize waist command positions if fixing
    if (enableWaist) {
        sdk.getMotorActual(actuals);
        for (int j = 0; j < dofWaist; ++j) {
            waistCmdPos[j] = actuals[waistIdx[j]].pos;
        }
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

        if (runArms) {
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
                const bool masterValid = isValidSw(actuals[mi].statusWord);
                const bool slaveValid = isValidSw(actuals[si].statusWord);
                if (!slaveValid) {
                    targets[si].pos = actuals[si].pos;
                    targets[si].enabled = 0;
                    continue;
                }
                if (relative && !arm0Valid[j] && masterValid) {
                    armMaster0[j] = actuals[mi].pos;
                    armSlave0[j] = actuals[si].pos;
                    arm0Valid[j] = 1;
                    slaveCmdPos[j] = actuals[si].pos;
                }
                if (!masterValid || (relative && !arm0Valid[j])) {
                    targets[si].pos = actuals[si].pos;
                    targets[si].enabled = enableSlave ? 1 : 0;
                    slaveCmdPos[j] = actuals[si].pos;
                    continue;
                }
                float desired = sign[j] * actuals[mi].pos + offset[j];
                if (relative) {
                    desired = armSlave0[j] + sign[j] * (actuals[mi].pos - armMaster0[j]) + offset[j];
                }
                slaveCmdPos[j] = slaveCmdPos[j] + armAlpha * (desired - slaveCmdPos[j]);
                targets[si].pos = slaveCmdPos[j];
                targets[si].enabled = enableSlave ? 1 : 0;
            }
        }
        
        if (runLegs) {
            // Legs: mirror master-side leg to the other side.
            for (int j = 0; j < dofLeg; ++j) {
                const int mi = masterLegIdx[j];
                const int si = slaveLegIdx[j];
                const bool masterValid = isValidSw(actuals[mi].statusWord);
                const bool slaveValid = isValidSw(actuals[si].statusWord);
                targets[mi].pos = actuals[mi].pos;
                targets[mi].enabled = enableMaster ? 1 : 0;

                if (!slaveValid) {
                    targets[si].pos = actuals[si].pos;
                    targets[si].enabled = 0;
                    continue;
                }
                if (relative && !leg0Valid[j] && masterValid) {
                    legMaster0[j] = actuals[mi].pos;
                    legSlave0[j] = actuals[si].pos;
                    leg0Valid[j] = 1;
                    legSlaveCmdPos[j] = actuals[si].pos;
                }
                if (!masterValid || (relative && !leg0Valid[j])) {
                    targets[si].pos = actuals[si].pos;
                    targets[si].enabled = enableSlave ? 1 : 0;
                    legSlaveCmdPos[j] = actuals[si].pos;
                    continue;
                }
                float desired = legSign[j] * actuals[mi].pos + legOffset[j];
                if (relative) {
                    desired = legSlave0[j] + legSign[j] * (actuals[mi].pos - legMaster0[j]) + legOffset[j];
                }
                legSlaveCmdPos[j] = legSlaveCmdPos[j] + legAlpha * (desired - legSlaveCmdPos[j]);
                targets[si].pos = legSlaveCmdPos[j];
                targets[si].enabled = enableSlave ? 1 : 0;
            }
        }
        
        // Waist fixing: hold waist motors at current position
        if (enableWaist) {
            for (int j = 0; j < dofWaist; ++j) {
                const int wi = waistIdx[j];
                targets[wi].pos = waistCmdPos[j];
                targets[wi].vel = 0.0f;
                targets[wi].tor = 0.0f;
                targets[wi].kp = 0.0f;
                targets[wi].kd = 0.0f;
                targets[wi].enabled = 1;
            }
        }

        sdk.setMotorTarget(targets);

        if ((tick++ % 500) == 0) {
            if (runArms && dofArm > 0) {
                const int j0 = 0;
                const int mi = masterIdx[j0];
                const int si = slaveIdx[j0];
                std::cout << "[tick " << tick << "]"
                          << " ARM M0 pos=" << actuals[mi].pos << " sw=0x" << std::hex << actuals[mi].statusWord
                          << std::dec << " en=" << targets[mi].enabled
                          << " | S0 cmd=" << targets[si].pos << " act=" << actuals[si].pos << " sw=0x" << std::hex
                          << actuals[si].statusWord << std::dec << " en=" << targets[si].enabled << "\n";
            }
            if (runLegs && dofLeg > 0) {
                const int j0 = 0;
                const int mi = masterLegIdx[j0];
                const int si = slaveLegIdx[j0];
                std::cout << "[tick " << tick << "]"
                          << " LEG M0 pos=" << actuals[mi].pos << " sw=0x" << std::hex << actuals[mi].statusWord
                          << std::dec << " en=" << targets[mi].enabled
                          << " | S0 cmd=" << targets[si].pos << " act=" << actuals[si].pos << " sw=0x" << std::hex
                          << actuals[si].statusWord << std::dec << " en=" << targets[si].enabled << "\n";
            }
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
