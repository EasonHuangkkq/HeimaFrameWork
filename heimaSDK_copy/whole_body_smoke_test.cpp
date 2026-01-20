// Build (Linux, requires EtherCAT library + headers in /usr/local):
//   # Without IMU:
//   g++ -std=c++20 -O2 -pthread \
//     whole_body_smoke_test.cpp heima_driver_sdk.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp rs485.cpp can.cpp \
//     -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
//     -o build/whole_body_smoke_test
//
//   # With YeSense IMU (RS232):
//   g++ -std=c++20 -O2 -pthread -DENABLE_RS232 \
//     whole_body_smoke_test.cpp heima_driver_sdk.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp \
//     rs232.cpp yesense_imu.cpp yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp \
//     rs485.cpp can.cpp \
//     -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
//     -o build/whole_body_smoke_test
//
// Run examples:
//   # 1) Monitor all motors (no commands):
//   sudo ./build/whole_body_smoke_test --config config.xml --all --disable
//
//   # 2) Enable and hold current position for all motors:
//   sudo ./build/whole_body_smoke_test --config config.xml --all --enable
//
//   # 3) Test only legs with sine wave on joint 0:
//   sudo ./build/whole_body_smoke_test --config config.xml --legs --enable --joint 0 --amp 0.05 --freq 0.2
//
//   # 4) Test left arm (limb 2) only:
//   sudo ./build/whole_body_smoke_test --config config.xml --limb 2 --enable
//
//   # 5) Test specific motors by alias:
//   sudo ./build/whole_body_smoke_test --config config.xml --aliases 1,2,3,4,5,6 --enable
//
//   # 6) Sequential testing (test joints one by one):
//   sudo ./build/whole_body_smoke_test --config config.xml --all --enable --sequential --joint 0
//
//   # 7) Monitor motors + IMU data:
//   sudo ./build/whole_body_smoke_test --config config.xml --all --enable --imu
//
//   # 8) Calibrate all motors (sets CountBias from current position):
//   sudo ./build/whole_body_smoke_test --config config.xml --all --calibrate
//
//   # 9) Calibrate left arm only and exit:
//   sudo ./build/whole_body_smoke_test --config config.xml --limb 2 --calibrate-only
//
// Notes:
// - This program supports testing whole robot (all 33 motors) or subsets by limb/alias.
// - Limb mapping: 0=left leg, 1=right leg, 2=left arm, 3=right arm, 4=waist, 5=neck
// - By default it does NOT enable drives; pass --enable to actually command motors.
// - Use --sequential to test joints one at a time for safer whole-body testing.
// - Use --imu to display IMU data (RPY angles, gyroscope, accelerometer).

#include "heima_driver_sdk.h"

#include "config_xml.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
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

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--config config.xml]\n"
        << "              [--all] [--limb N] [--aliases a1,a2,...]\n"
        << "              [--legs] [--arms] [--waist] [--neck]\n"
        << "              [--mode 8] [--period-us 1000] [--seconds 10]\n"
        << "              [--enable] [--disable] [--follow-actual] [--settle-ms 0]\n"
        << "              [--joint j] [--amp rad] [--freq hz]\n"
        << "              [--sequential] [--sequential-delay-ms 1000]\n"
        << "              [--print-ms 200] [--max-current u16]\n"
        << "              [--imu]\n"
        << "\n"
        << "Motor Selection (one required):\n"
        << "  --all          Select ALL motors from all limbs\n"
        << "  --limb N       Select limb N (0=LL,1=RL,2=LA,3=RA,4=waist,5=neck)\n"
        << "  --aliases a1,a2,...  Select motors by explicit aliases (1-based)\n"
        << "  --legs         Select both left and right legs (limb 0,1)\n"
        << "  --arms         Select both left and right arms (limb 2,3)\n"
        << "  --waist        Select waist (limb 4)\n"
        << "  --neck         Select neck (limb 5)\n"
        << "\n"
        << "Test Modes:\n"
        << "  --mode         CiA-402 mode (default: 8 CSP)\n"
        << "  --period-us    Loop period in microseconds (default: 1000)\n"
        << "  --seconds      Run time after reaching OP (default: 10, <0 means forever)\n"
        << "  --enable       Actually enable drives (default: disabled/monitor-only)\n"
        << "  --disable      Monitor only (explicitly disables commanding)\n"
        << "  --follow-actual  Each cycle set target position to actual (no locking)\n"
        << "  --settle-ms    Follow actual for N ms after OP, then freeze base position\n"
        << "  --sequential   Test selected joints one by one (for safer whole-body testing)\n"
        << "  --sequential-delay-ms  Delay between joint tests in ms (default: 1000)\n"
        << "\n"
        << "Calibration:\n"
        << "  --calibrate     Run SDO-based CountBias calibration on selected motors\n"
        << "  --calibrate-only  Calibrate then exit without running test loop\n"
        << "\n"
        << "Sine Test (when --joint is set):\n"
        << "  --joint        Joint index within selected group (0-based) or all (-1)\n"
        << "                 Use --joint -1 to test ALL selected joints with sine wave\n"
        << "  --amp          Sine amplitude in rad (default: 0.05 if --joint is set)\n"
        << "  --freq         Sine frequency in Hz (default: 0.2 if --joint is set)\n"
        << "\n"
        << "Sensors:\n"
        << "  --imu          Display IMU data (RPY, gyroscope, accelerometer) in each cycle\n"
        << "\n"
        << "Other:\n"
        << "  --print-ms     Print period in ms (default: 200)\n"
        << "  --max-current  Override MaxTorque/MaxCurrent (U16) for ALL motors\n";
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

int clampInt(int v, int lo, int hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void fillHoldTargets(const std::vector<DriverSDK::motorActualStruct>& actuals,
                     std::vector<DriverSDK::motorTargetStruct>& targets) {
    const int motorCount = static_cast<int>(targets.size());
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = actuals[i].pos;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].kp = 0.0f;
        targets[i].kd = 0.0f;
        targets[i].enabled = 0;
    }
}

void printMotors(const std::string& title,
                  int tick,
                  const std::vector<int>& motorIdx,
                  const std::vector<int>& motorAlias,
                  const std::vector<std::string>& motorName,
                  const std::vector<DriverSDK::motorActualStruct>& actuals,
                  const std::vector<DriverSDK::motorTargetStruct>* targets) {
    std::cout << title << " tick=" << tick << " motor_count=" << motorIdx.size() << "\n";
    const int maxDisplay = 20;
    for (int j = 0; j < std::min(static_cast<int>(motorIdx.size()), maxDisplay); ++j) {
        const int idx = motorIdx[j];
        const unsigned short sw = actuals[idx].statusWord;
        std::cout << "  [" << motorName[j] << "] alias=" << motorAlias[j] << " idx=" << idx
                  << " pos=" << actuals[idx].pos
                  << " vel=" << actuals[idx].vel
                  << " tor=" << actuals[idx].tor
                  << " sw=0x" << std::hex << sw << std::dec;
        if (targets != nullptr) {
            std::cout << " cmd_pos=" << (*targets)[idx].pos
                      << " en=" << (*targets)[idx].enabled;
        }
        std::cout << "\n";
    }
    if (static_cast<int>(motorIdx.size()) > maxDisplay) {
        std::cout << "  ... (" << (motorIdx.size() - maxDisplay) << " more motors)\n";
    }
}

void printIMU(const std::string& title, const DriverSDK::imuStruct& imu) {
    std::cout << title << "\n";
    std::cout << "  RP Y [rad]: " << imu.rpy[0] << " " << imu.rpy[1] << " " << imu.rpy[2] << "\n";
    std::cout << "  Gyro [rad/s]: " << imu.gyr[0] << " " << imu.gyr[1] << " " << imu.gyr[2] << "\n";
    std::cout << "  Acc  [m/s^2]: " << imu.acc[0] << " " << imu.acc[1] << " " << imu.acc[2] << "\n";
}

std::string getLimbName(int limb) {
    switch (limb) {
        case 0: return "LL";
        case 1: return "RL";
        case 2: return "LA";
        case 3: return "RA";
        case 4: return "Waist";
        case 5: return "Neck";
        default: return "L" + std::to_string(limb);
    }
}

bool isOp(unsigned short statusWord) {
    return (statusWord & 0x007f) == 0x0037;
}

void disableAllAndExit(DriverSDK::DriverSDK& sdk,
                      const std::vector<int>& motorIdx,
                      int periodUs,
                      std::vector<DriverSDK::motorActualStruct>& actuals,
                      std::vector<DriverSDK::motorTargetStruct>& targets) {
    sdk.getMotorActual(actuals);
    fillHoldTargets(actuals, targets);
    for (int idx : motorIdx) {
        targets[idx].enabled = 0;
    }
    for (int i = 0; i < 200; ++i) {
        sdk.setMotorTarget(targets);
        usleep(static_cast<useconds_t>(periodUs));
    }
}
}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    bool selectAll = false;
    int limb = -1;
    std::string aliasesCsv;
    bool selectLegs = false;
    bool selectArms = false;
    bool selectWaist = false;
    bool selectNeck = false;
    bool enableImu = false;
    bool calibrate = false;
    bool calibrateOnly = false;

    int modeInt = 8;
    int periodUs = 1000;
    double seconds = 10.0;
    bool enable = false;
    bool followActual = false;
    int settleMs = 0;
    int joint = -1;
    bool jointSet = false;
    double amp = 0.0;
    bool ampSet = false;
    double freq = 0.0;
    bool freqSet = false;
    bool sequential = false;
    int sequentialDelayMs = 1000;
    int printMs = 200;
    int maxCurrentU16 = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
            continue;
        }
        if (arg == "--all") {
            selectAll = true;
            continue;
        }
        if (arg == "--limb" && i + 1 < argc) {
            limb = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--aliases" && i + 1 < argc) {
            aliasesCsv = argv[++i];
            continue;
        }
        if (arg == "--legs") {
            selectLegs = true;
            continue;
        }
        if (arg == "--arms") {
            selectArms = true;
            continue;
        }
        if (arg == "--waist") {
            selectWaist = true;
            continue;
        }
        if (arg == "--neck") {
            selectNeck = true;
            continue;
        }
        if (arg == "--imu") {
            enableImu = true;
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
        if (arg == "--mode" && i + 1 < argc) {
            modeInt = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--period-us" && i + 1 < argc) {
            periodUs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--enable") {
            enable = true;
            continue;
        }
        if (arg == "--disable") {
            enable = false;
            continue;
        }
        if (arg == "--follow-actual") {
            followActual = true;
            continue;
        }
        if (arg == "--settle-ms" && i + 1 < argc) {
            settleMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--joint" && i + 1 < argc) {
            joint = std::stoi(argv[++i]);
            jointSet = true;
            continue;
        }
        if (arg == "--amp" && i + 1 < argc) {
            amp = std::stod(argv[++i]);
            ampSet = true;
            continue;
        }
        if (arg == "--freq" && i + 1 < argc) {
            freq = std::stod(argv[++i]);
            freqSet = true;
            continue;
        }
        if (arg == "--sequential") {
            sequential = true;
            continue;
        }
        if (arg == "--sequential-delay-ms" && i + 1 < argc) {
            sequentialDelayMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--print-ms" && i + 1 < argc) {
            printMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--max-current" && i + 1 < argc) {
            maxCurrentU16 = std::stoi(argv[++i]);
            continue;
        }
        if (!arg.empty() && arg[0] != '-') {
            configPath = arg;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << "\n";
        printUsage(argv[0]);
        return 1;
    }

    if (periodUs <= 0) {
        periodUs = 1000;
    }
    if (printMs <= 0) {
        printMs = 200;
    }
    const int printEvery = std::max(1, (printMs * 1000 + periodUs - 1) / periodUs);
    if (settleMs < 0) {
        settleMs = 0;
    }
    if (sequentialDelayMs < 0) {
        sequentialDelayMs = 1000;
    }
    maxCurrentU16 = (maxCurrentU16 < 0) ? -1 : clampInt(maxCurrentU16, 0, 65535);

    if (jointSet && joint < -1) {
        std::cerr << "Invalid --joint: " << joint << " (must be >=0 or -1)\n";
        return 1;
    }
    if (jointSet) {
        if (!ampSet) {
            amp = 0.05;
        }
        if (!freqSet) {
            freq = 0.2;
        }
    }
    if (amp < 0.0) {
        amp = -amp;
    }
    if (freq < 0.0) {
        freq = -freq;
    }
    if (calibrate && !enable) {
        std::cout << "--calibrate implies --enable\n";
        enable = true;
    }
    if (!sequential && followActual && jointSet) {
        std::cerr << "--follow-actual conflicts with --joint (sine). Remove one.\n";
        return 1;
    }
    if (sequential && jointSet && joint >= 0) {
        std::cerr << "--sequential with specific joint: will test only that joint. Use --joint -1 or omit --joint to test all.\n";
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    if (motorCount <= 0) {
        std::cerr << "No motors configured. Check <Motors> in " << configPath << "\n";
        return 1;
    }

    std::cout << "Config: " << configPath << " total_motors=" << motorCount << "\n";

    sdk.setMode(std::vector<char>(motorCount, static_cast<char>(modeInt)));

    if (maxCurrentU16 >= 0) {
        sdk.setMaxCurr(std::vector<unsigned short>(motorCount, static_cast<unsigned short>(maxCurrentU16)));
    }

    std::vector<int> selectedAlias;
    std::string selectionDesc;

    if (!aliasesCsv.empty()) {
        selectedAlias = parseCsvInts(aliasesCsv);
        selectionDesc = "aliases=" + aliasesCsv;
    } else {
        DriverSDK::ConfigXML cfg(configPath.c_str());
        const std::vector<std::vector<int>> motorAlias = cfg.motorAlias();

        if (motorAlias.size() != 6) {
            std::cerr << "Invalid config.xml: expected 6 limbs, got " << motorAlias.size() << "\n";
            return 1;
        }

        std::vector<int> limbsToSelect;
        if (selectAll) {
            for (int l = 0; l < 6; ++l) {
                limbsToSelect.push_back(l);
            }
            selectionDesc = "all_limbs";
        } else if (limb >= 0) {
            if (limb >= 6) {
                std::cerr << "Invalid limb index: " << limb << " (must be 0-5)\n";
                return 1;
            }
            limbsToSelect.push_back(limb);
            selectionDesc = "limb=" + getLimbName(limb);
        } else {
            if (selectLegs) {
                limbsToSelect.push_back(0);
                limbsToSelect.push_back(1);
            }
            if (selectArms) {
                limbsToSelect.push_back(2);
                limbsToSelect.push_back(3);
            }
            if (selectWaist) {
                limbsToSelect.push_back(4);
            }
            if (selectNeck) {
                limbsToSelect.push_back(5);
            }

            if (limbsToSelect.empty()) {
                std::cerr << "No motors selected. Use --all, --limb, --legs, --arms, --waist, --neck, or --aliases.\n";
                printUsage(argv[0]);
                return 1;
            }

            std::string limbDesc;
            for (size_t i = 0; i < limbsToSelect.size(); ++i) {
                if (i > 0) limbDesc += ",";
                limbDesc += getLimbName(limbsToSelect[i]);
            }
            selectionDesc = "limbs=" + limbDesc;
        }

        for (int l : limbsToSelect) {
            if (l < 0 || l >= static_cast<int>(motorAlias.size())) {
                std::cerr << "Invalid limb index: " << l << "\n";
                return 1;
            }
            for (int a : motorAlias[l]) {
                if (a > 0) {
                    selectedAlias.push_back(a);
                }
            }
        }
    }

    std::sort(selectedAlias.begin(), selectedAlias.end());
    selectedAlias.erase(std::unique(selectedAlias.begin(), selectedAlias.end()), selectedAlias.end());

    std::vector<int> motorIdx;
    std::vector<int> motorAliasForPrint;
    std::vector<std::string> motorName;

    for (int a : selectedAlias) {
        if (a <= 0) {
            continue;
        }
        if (a > motorCount) {
            std::cerr << "Motor alias out of range: alias=" << a << " motorCount=" << motorCount << "\n";
            return 1;
        }
        motorIdx.push_back(a - 1);
        motorAliasForPrint.push_back(a);
        motorName.push_back("m" + std::to_string(a));
    }
    if (motorIdx.empty()) {
        std::cerr << "No motors selected. Check config.xml limb mapping or use --aliases.\n";
        return 1;
    }
    if (jointSet && joint >= 0 && joint >= static_cast<int>(motorIdx.size())) {
        std::cerr << "Joint index out of range: joint=" << joint << " selected_count=" << motorIdx.size() << "\n";
        return 1;
    }

    std::cout << "Selection: " << selectionDesc << " count=" << motorIdx.size() << "\n";
    std::cout << "Mode: " << modeInt << " period_us: " << periodUs
              << " enable: " << (enable ? "yes" : "no")
              << " seconds: " << seconds << "\n";
    if (enable) {
        std::cout << "Follow actual: " << (followActual ? "yes" : "no")
                  << " settle_ms: " << settleMs << "\n";
        std::cout << "Sequential: " << (sequential ? "yes" : "no");
        if (sequential) {
            std::cout << " delay_ms: " << sequentialDelayMs;
        }
        std::cout << "\n";
    }
    std::cout << "IMU enabled: " << (enableImu ? "yes" : "no") << "\n";
    if (jointSet) {
        if (joint == -1) {
            std::cout << "Sine test: ALL joints amp=" << amp << " rad freq=" << freq << " Hz\n";
        } else {
            std::cout << "Sine test: joint=" << joint << " amp=" << amp << " rad freq=" << freq << " Hz\n";
        }
    }
    if (maxCurrentU16 >= 0) {
        std::cout << "MaxCurrent/MaxTorque override: " << maxCurrentU16 << " (u16)\n";
    }

    for (int j = 0; j < std::min(static_cast<int>(motorIdx.size()), 5); ++j) {
        std::cout << "  [" << motorName[j] << "] alias=" << motorAliasForPrint[j] << " idx=" << motorIdx[j] << "\n";
    }
    if (motorIdx.size() > 5) {
        std::cout << "  ... (" << (motorIdx.size() - 5) << " more)\n";
    }

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    DriverSDK::imuStruct imu{};

    if (!enable) {
        int tick = 0;
        const double maxTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            if (enableImu) {
                sdk.getIMU(imu);
            }
            if ((tick % printEvery) == 0) {
                printMotors("[MON]", tick, motorIdx, motorAliasForPrint, motorName, actuals, nullptr);
                if (enableImu) {
                    printIMU("[IMU]", imu);
                }
            }
            if (maxTicks >= 0.0 && tick >= static_cast<int>(maxTicks)) {
                break;
            }
            ++tick;
            usleep(static_cast<useconds_t>(periodUs));
        }
        return 0;
    }

    const int maxWaitTicks = (int)(10.0 * 1e6 / static_cast<double>(periodUs));
    int waitTick = 0;
    while (!g_stop) {
        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets);
        for (int idx : motorIdx) {
            targets[idx].enabled = 1;
        }
        sdk.setMotorTarget(targets);

        bool allOp = true;
        for (int idx : motorIdx) {
            if (!isOp(actuals[idx].statusWord)) {
                allOp = false;
                break;
            }
        }
        if ((waitTick % printEvery) == 0) {
            printMotors("[WAIT_OP]", waitTick, motorIdx, motorAliasForPrint, motorName, actuals, &targets);
        }
        if (allOp) {
            break;
        }
        if (++waitTick > maxWaitTicks) {
            std::cerr << "Timeout waiting for OP (0x0037).\n";
            break;
        }
        usleep(static_cast<useconds_t>(periodUs));
    }

    if (g_stop) {
        disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets);
        return 0;
    }

    if (calibrate) {
        std::cout << "\n=== Calibration: place robot at zero pose and keep still ===\n";
        const int maxWaitTicks = (int)(10.0 * 1e6 / static_cast<double>(periodUs));
        int waitTick = 0;
        bool allOp = false;
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            fillHoldTargets(actuals, targets);
            for (int idx : motorIdx) {
                targets[idx].enabled = 1;
            }
            sdk.setMotorTarget(targets);

            allOp = true;
            for (int idx : motorIdx) {
                if (!isOp(actuals[idx].statusWord)) {
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

        for (int alias : motorAliasForPrint) {
            if (alias > 0) {
                calibrateAlias(alias);
            }
        }

        if (calibrateOnly) {
            disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets);
            return 0;
        }
    }

    sdk.getMotorActual(actuals);

    std::vector<float> basePos(motorIdx.size(), 0.0f);
    for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
        basePos[j] = actuals[motorIdx[j]].pos;
    }

    if (!followActual && settleMs > 0) {
        const int settleTicks = std::max(1, (settleMs * 1000 + periodUs - 1) / periodUs);
        for (int s = 0; s < settleTicks && !g_stop; ++s) {
            sdk.getMotorActual(actuals);
            fillHoldTargets(actuals, targets);
            for (int idx : motorIdx) {
                targets[idx].enabled = 1;
            }
            sdk.setMotorTarget(targets);
            if ((s % printEvery) == 0) {
                printMotors("[SETTLE]", s, motorIdx, motorAliasForPrint, motorName, actuals, &targets);
            }
            usleep(static_cast<useconds_t>(periodUs));
        }
        sdk.getMotorActual(actuals);
        for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
            basePos[j] = actuals[motorIdx[j]].pos;
        }
    }

    const double twoPi = 2.0 * std::acos(-1.0);
    int tick = 0;
    const double maxTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));

    if (sequential) {
        std::vector<int> testIndices;
        if (jointSet && joint >= 0) {
            testIndices.push_back(joint);
        } else {
            for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
                testIndices.push_back(j);
            }
        }

        for (int testIdx : testIndices) {
            std::cout << "\n=== Testing joint " << testIdx << " (alias=" << motorAliasForPrint[testIdx]
                      << ") ===\n";

            for (int seqTick = 0; seqTick < sequentialDelayMs && !g_stop; ++seqTick) {
                sdk.getMotorActual(actuals);
                fillHoldTargets(actuals, targets);
                for (int idx : motorIdx) {
                    targets[idx].enabled = 1;
                }
                const int activeIdx = motorIdx[testIdx];
                targets[activeIdx].pos = actuals[activeIdx].pos;
                sdk.setMotorTarget(targets);
                usleep(1000);
            }

            int seqTick = 0;
            const double maxSeqTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));
            while (!g_stop) {
                sdk.getMotorActual(actuals);
                if (enableImu) {
                    sdk.getIMU(imu);
                }
                fillHoldTargets(actuals, targets);

                for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
                    const int idx = motorIdx[j];
                    targets[idx].enabled = 1;
                    if (j == testIdx) {
                        targets[idx].pos = basePos[testIdx];
                        if (amp > 0.0 && freq > 0.0) {
                            const double t = static_cast<double>(seqTick) * static_cast<double>(periodUs) * 1e-6;
                            targets[idx].pos = (float)(basePos[testIdx] + amp * std::sin(twoPi * freq * t));
                        }
                    } else {
                        targets[idx].pos = actuals[idx].pos;
                    }
                }
                sdk.setMotorTarget(targets);

                if ((seqTick % printEvery) == 0) {
                    std::cout << "[SEQ] test_j=" << testIdx << " tick=" << seqTick
                              << " m" << motorAliasForPrint[testIdx]
                              << " pos=" << actuals[motorIdx[testIdx]].pos
                              << " cmd=" << targets[motorIdx[testIdx]].pos << "\n";
                    if (enableImu) {
                        printIMU("[SEQ-IMU]", imu);
                    }
                }
                if (maxSeqTicks >= 0.0 && seqTick >= static_cast<int>(maxSeqTicks)) {
                    break;
                }
                ++seqTick;
                usleep(static_cast<useconds_t>(periodUs));
            }

            if (g_stop) {
                break;
            }
        }
    } else {
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            if (enableImu) {
                sdk.getIMU(imu);
            }
            fillHoldTargets(actuals, targets);

            for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
                const int idx = motorIdx[j];
                targets[idx].enabled = 1;
                float desired = targets[idx].pos;
                if (!followActual) {
                    desired = basePos[j];
                    if (jointSet && (joint == -1 || j == joint) && amp > 0.0 && freq > 0.0) {
                        const double t = static_cast<double>(tick) * static_cast<double>(periodUs) * 1e-6;
                        desired = (float)(basePos[j] + amp * std::sin(twoPi * freq * t));
                    }
                }
                targets[idx].pos = desired;
            }
            sdk.setMotorTarget(targets);

            if ((tick % printEvery) == 0) {
                printMotors("[RUN]", tick, motorIdx, motorAliasForPrint, motorName, actuals, &targets);
                if (enableImu) {
                    printIMU("[IMU]", imu);
                }
            }
            if (maxTicks >= 0.0 && tick >= static_cast<int>(maxTicks)) {
                break;
            }
            ++tick;
            usleep(static_cast<useconds_t>(periodUs));
        }
    }

    disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets);
    return 0;
}
