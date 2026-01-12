// Build (Linux, requires EtherCAT library + headers in /usr/local):
// g++ -std=c++20 -O2 -pthread \
//   arm_smoke_test.cpp heima_driver_sdk.cpp ecat.cpp config_xml.cpp common.cpp tinyxml2.cpp rs232.cpp rs485.cpp can.cpp \
//   -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
//   -o build/arm_smoke_test
//
// Run examples:
//   # 1) Just monitor (no commands sent):
//   sudo ./build/arm_smoke_test --config config.xml --limb 2
//
//   # 2) Enable and hold current position (safe default):
//   sudo ./build/arm_smoke_test --config config.xml --limb 2 --enable
//
//   # 3) Enable and do a small sine test on joint 0:
//   sudo ./build/arm_smoke_test --config config.xml --limb 2 --enable --joint 0 --amp 0.05 --freq 0.2
//
// Notes:
// - This program targets ONE limb (e.g. a single arm) and ignores digits/fingers.
// - It always "hold current position" unless you explicitly ask for motion.
// - By default it does NOT enable drives; pass --enable to actually command motors.

#include "heima_driver_sdk.h"

#include "config_xml.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
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
        << "Usage: " << argv0 << " [--config config.xml] [--limb 2] [--aliases a1,a2,...]\n"
        << "              [--mode 8] [--period-us 1000] [--seconds 10]\n"
        << "              [--enable] [--follow-actual] [--settle-ms 0]\n"
        << "              [--joint j] [--amp rad] [--freq hz]\n"
        << "              [--print-ms 200] [--max-current u16]\n"
        << "\n"
        << "  --config       Path to config.xml (default: config.xml)\n"
        << "  --limb         Limb index in config.xml <Motors> (default: auto)\n"
        << "                (Typical: 2=left arm, 3=right arm, 4=waist, 5=neck)\n"
        << "  --aliases      Override limb selection with explicit motor aliases (1-based)\n"
        << "  --mode         CiA-402 mode (default: 8 CSP)\n"
        << "  --period-us    Loop period in microseconds (default: 1000)\n"
        << "  --seconds      Run time after reaching OP (default: 10, <0 means forever)\n"
        << "  --enable       Actually enable drives (default: disabled/monitor-only)\n"
        << "  --follow-actual  Each cycle set target position to actual (no locking)\n"
        << "  --settle-ms    Follow actual for N ms after OP, then freeze base position\n"
        << "  --joint        Arm joint index within selected group (0-based); enables sine test\n"
        << "  --amp          Sine amplitude in rad (default: 0.05 if --joint is set)\n"
        << "  --freq         Sine frequency in Hz (default: 0.2 if --joint is set)\n"
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
                 const std::vector<DriverSDK::motorActualStruct>& actuals,
                 const std::vector<DriverSDK::motorTargetStruct>* targets) {
    std::cout << title << " tick=" << tick << "\n";
    for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
        const int idx = motorIdx[j];
        const unsigned short sw = actuals[idx].statusWord;
        std::cout << "  j" << j << " alias=" << motorAlias[j] << " idx=" << idx
                  << " pos=" << actuals[idx].pos
                  << " vel=" << actuals[idx].vel
                  << " tor=" << actuals[idx].tor
                  << " sw=0x" << std::hex << sw << std::dec
                  << " state=0x" << std::hex << (sw & 0x007f) << std::dec
                  << " err=0x" << std::hex << actuals[idx].errorCode << std::dec;
        if (targets != nullptr) {
            std::cout << " cmd_pos=" << (*targets)[idx].pos
                      << " en=" << (*targets)[idx].enabled;
        }
        std::cout << "\n";
    }
}
}

int main(int argc, char** argv) {
    std::string configPath = "config.xml";
    int limb = -1;
    std::string aliasesCsv;
    int modeInt = 8;
    int periodUs = 1000;
    double seconds = 10.0;
    bool enable = false;
    bool followActual = false;
    int settleMs = 0;
    int joint = -1;
    double amp = 0.0;
    bool ampSet = false;
    double freq = 0.0;
    bool freqSet = false;
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
        if (arg == "--limb" && i + 1 < argc) {
            limb = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--aliases" && i + 1 < argc) {
            aliasesCsv = argv[++i];
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
    maxCurrentU16 = (maxCurrentU16 < 0) ? -1 : clampInt(maxCurrentU16, 0, 65535);

    if (joint >= 0) {
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
    if (followActual && joint >= 0) {
        std::cerr << "--follow-actual conflicts with --joint (sine). Remove one.\n";
        return 1;
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

    // Configure mode for all motors (simplest for smoke tests).
    sdk.setMode(std::vector<char>(motorCount, static_cast<char>(modeInt)));

    if (maxCurrentU16 >= 0) {
        sdk.setMaxCurr(std::vector<unsigned short>(motorCount, static_cast<unsigned short>(maxCurrentU16)));
    }

    // Select which motors to test: either by explicit aliases or by limb in config.xml.
    std::vector<int> selectedAlias;
    if (!aliasesCsv.empty()) {
        selectedAlias = parseCsvInts(aliasesCsv);
    } else {
        DriverSDK::ConfigXML cfg(configPath.c_str());
        const std::vector<std::vector<int>> motorAlias = cfg.motorAlias();

        if (limb < 0) {
            const int preferOrder[] = {2, 3, 4, 5, 0, 1};
            for (int k = 0; k < static_cast<int>(sizeof(preferOrder) / sizeof(preferOrder[0])); ++k) {
                const int cand = preferOrder[k];
                if (cand >= 0 && cand < static_cast<int>(motorAlias.size()) && !motorAlias[cand].empty()) {
                    bool hasValid = false;
                    for (int a : motorAlias[cand]) {
                        if (a > 0) {
                            hasValid = true;
                            break;
                        }
                    }
                    if (hasValid) {
                        limb = cand;
                        break;
                    }
                }
            }
        }

        if (limb < 0 || limb >= static_cast<int>(motorAlias.size())) {
            std::cerr << "No limb motors found in config.xml. Use --aliases to specify motors.\n";
            return 1;
        }
        selectedAlias = motorAlias[limb];
    }

    std::vector<int> motorIdx;
    std::vector<int> motorAliasForPrint;
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
    }
    if (motorIdx.empty()) {
        std::cerr << "No motors selected. Check config.xml limb mapping or use --aliases.\n";
        return 1;
    }

    if (joint >= 0 && joint >= static_cast<int>(motorIdx.size())) {
        std::cerr << "--joint out of range: joint=" << joint << " dof=" << motorIdx.size() << "\n";
        return 1;
    }

    std::cout << "Config: " << configPath << "\n";
    if (!aliasesCsv.empty()) {
        std::cout << "Selection: aliases=" << aliasesCsv << "\n";
    } else {
        std::cout << "Selection: limb=" << limb << "\n";
    }
    std::cout << "Motor count: " << motorCount << " selected: " << motorIdx.size() << "\n";
    std::cout << "Mode: " << modeInt << " period_us: " << periodUs
              << " enable: " << (enable ? "yes" : "no")
              << " seconds: " << seconds << "\n";
    if (enable) {
        std::cout << "Follow actual: " << (followActual ? "yes" : "no")
                  << " settle_ms: " << settleMs << "\n";
    }
    if (joint >= 0) {
        std::cout << "Sine test: joint=" << joint << " amp=" << amp << " rad freq=" << freq << " Hz\n";
    }
    if (maxCurrentU16 >= 0) {
        std::cout << "MaxCurrent/MaxTorque override: " << maxCurrentU16 << " (u16)\n";
    }
    for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
        std::cout << "  j" << j << ": alias=" << motorAliasForPrint[j] << " idx=" << motorIdx[j] << "\n";
    }

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);

    if (!enable) {
        int tick = 0;
        const double maxTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            if ((tick % printEvery) == 0) {
                printMotors("[MON]", tick, motorIdx, motorAliasForPrint, actuals, nullptr);
            }
            if (maxTicks >= 0.0 && tick >= static_cast<int>(maxTicks)) {
                break;
            }
            ++tick;
            usleep(static_cast<useconds_t>(periodUs));
        }
        return 0;
    }

    // Enable path:
    // 1) Wait until all selected motors reach OP (state bits == 0x0037), while holding current position.
    const int maxWaitTicks = (int)(10.0 * 1e6 / static_cast<double>(periodUs)); // ~10s
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
            if ((actuals[idx].statusWord & 0x007f) != 0x0037) {
                allOp = false;
            }
        }
        if ((waitTick % printEvery) == 0) {
            printMotors("[WAIT_OP]", waitTick, motorIdx, motorAliasForPrint, actuals, &targets);
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
        // Fall through to disable section.
    } else {
        // Re-sample base positions AFTER reaching OP to avoid steps.
        sdk.getMotorActual(actuals);
    }

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
                printMotors("[SETTLE]", s, motorIdx, motorAliasForPrint, actuals, &targets);
            }
            usleep(static_cast<useconds_t>(periodUs));
        }
        sdk.getMotorActual(actuals);
        for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
            basePos[j] = actuals[motorIdx[j]].pos;
        }
    }

    // 2) Run hold/sine loop for requested duration.
    const double twoPi = 2.0 * std::acos(-1.0);
    int tick = 0;
    const double maxTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));
    while (!g_stop) {
        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets);
        for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
            const int idx = motorIdx[j];
            targets[idx].enabled = 1;
            float desired = targets[idx].pos;
            if (!followActual) {
                desired = basePos[j];
                if (joint >= 0 && j == joint && amp > 0.0 && freq > 0.0) {
                    const double t = static_cast<double>(tick) * static_cast<double>(periodUs) * 1e-6;
                    desired = (float)(basePos[j] + amp * std::sin(twoPi * freq * t));
                }
            }
            targets[idx].pos = desired;
        }
        sdk.setMotorTarget(targets);

        if ((tick % printEvery) == 0) {
            printMotors("[RUN]", tick, motorIdx, motorAliasForPrint, actuals, &targets);
        }
        if (maxTicks >= 0.0 && tick >= static_cast<int>(maxTicks)) {
            break;
        }
        ++tick;
        usleep(static_cast<useconds_t>(periodUs));
    }

    // 3) Disable all drives on exit (send a few cycles to apply).
    sdk.getMotorActual(actuals);
    fillHoldTargets(actuals, targets);
    for (int idx : motorIdx) {
        targets[idx].enabled = 0;
    }
    for (int i = 0; i < 200; ++i) {
        sdk.setMotorTarget(targets);
        usleep(static_cast<useconds_t>(periodUs));
    }
    return 0;
}
