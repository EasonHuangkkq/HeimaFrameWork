#include "heima_driver_sdk.h"
#include "config_xml.h"

#include <algorithm>
#include <cctype>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;

void handleSignal(int) {
    g_stop = 1;
}

struct NotSpace {
    bool operator()(unsigned char c) const {
        return !std::isspace(c);
    }
};

std::string trim(std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), NotSpace()));
    s.erase(std::find_if(s.rbegin(), s.rend(), NotSpace()).base(), s.end());
    return s;
}

std::vector<int> parseCsvInts(const std::string& s) {
    std::vector<int> out;
    size_t start = 0;
    while (start < s.size()) {
        size_t end = s.find(',', start);
        std::string token = (end == std::string::npos) ? s.substr(start) : s.substr(start, end - start);
        token = trim(token);
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

bool isOp(unsigned short statusWord) {
    return (statusWord & 0x007f) == 0x0037;
}

struct TrajFrame {
    double tSec;
    std::vector<float> q;
    TrajFrame() : tSec(0.0), q() {}
};

bool tryParseCsvRow(const std::string& line, std::vector<std::string>& colsOut) {
    colsOut.clear();
    std::string s = trim(line);
    if (s.empty()) {
        return false;
    }
    if (!s.empty() && s[0] == '#') {
        return false;
    }

    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        colsOut.push_back(trim(item));
    }
    while (!colsOut.empty() && colsOut.back().empty()) {
        colsOut.pop_back();
    }
    return !colsOut.empty();
}

bool parseDouble(const std::string& s, double& v) {
    try {
        size_t idx = 0;
        v = std::stod(s, &idx);
        return idx == s.size();
    } catch (...) {
        return false;
    }
}

bool parseFloat(const std::string& s, float& v) {
    try {
        size_t idx = 0;
        v = std::stof(s, &idx);
        return idx == s.size();
    } catch (...) {
        return false;
    }
}

bool loadTrajectoryCsv(const std::string& path, int dofArm, std::vector<TrajFrame>& framesOut, bool& hasTimeOut) {
    framesOut.clear();
    hasTimeOut = false;

    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Failed to open trajectory file: " << path << "\n";
        return false;
    }

    std::string line;
    std::vector<std::string> cols;

    bool formatDecided = false;
    bool hasTime = false;
    bool maybeHeaderChecked = false;
    double lastT = -1e100;

    while (std::getline(in, line)) {
        if (!tryParseCsvRow(line, cols)) {
            continue;
        }

        if (!maybeHeaderChecked) {
            maybeHeaderChecked = true;
            double tmp;
            if (!parseDouble(cols[0], tmp)) {
                continue;
            }
        }

        if (!formatDecided) {
            if ((int)cols.size() == dofArm + 1) {
                hasTime = true;
            } else if ((int)cols.size() == dofArm) {
                hasTime = false;
            } else {
                std::cerr << "Trajectory row has wrong column count: got " << cols.size() << ", expected " << dofArm
                          << " or " << (dofArm + 1) << "\n";
                return false;
            }
            formatDecided = true;
        }

        TrajFrame f;
        f.q.resize(dofArm);

        if (hasTime) {
            if ((int)cols.size() != dofArm + 1) {
                std::cerr << "Mixed trajectory formats (time column present sometimes).\n";
                return false;
            }
            double t;
            if (!parseDouble(cols[0], t)) {
                std::cerr << "Invalid time value: " << cols[0] << "\n";
                return false;
            }
            if (t < lastT) {
                std::cerr << "Trajectory time must be non-decreasing. Got " << t << " after " << lastT << "\n";
                return false;
            }
            lastT = t;
            f.tSec = t;
            for (int j = 0; j < dofArm; ++j) {
                float q;
                if (!parseFloat(cols[j + 1], q)) {
                    std::cerr << "Invalid q value at column " << (j + 1) << ": " << cols[j + 1] << "\n";
                    return false;
                }
                f.q[j] = q;
            }
        } else {
            if ((int)cols.size() != dofArm) {
                std::cerr << "Mixed trajectory formats (time column missing sometimes).\n";
                return false;
            }
            f.tSec = 0.0;
            for (int j = 0; j < dofArm; ++j) {
                float q;
                if (!parseFloat(cols[j], q)) {
                    std::cerr << "Invalid q value at column " << j << ": " << cols[j] << "\n";
                    return false;
                }
                f.q[j] = q;
            }
        }

        framesOut.push_back(f);
    }

    if (framesOut.empty()) {
        std::cerr << "Trajectory file has no samples: " << path << "\n";
        return false;
    }

    hasTimeOut = hasTime;
    return true;
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--config config.xml] [--mode 8] [--period-us 1000]\n"
        << "              [--enable-right] [--enable-left] [--right a1,a2,...] [--left b1,b2,...]\n"
        << "              [--record-right out.csv] [--record-only]\n"
        << "              [--play in.csv] [--blend-ms 1000]\n"
        << "              [--wait-op-timeout-s 30] [--wait-all-motors] [--start-delay-ms 0] [--play-rate 1.0] [--play-loop]\n";
}

void disableAllAndExit(DriverSDK::DriverSDK& sdk,
                      int motorCount,
                      int periodUs,
                      std::vector<DriverSDK::motorActualStruct>& actuals,
                      std::vector<DriverSDK::motorTargetStruct>& targets) {
    sdk.getMotorActual(actuals);
    fillHoldTargets(actuals, targets);
    for (int i = 0; i < motorCount; ++i) {
        targets[i].enabled = 0;
    }
    for (int i = 0; i < 200; ++i) {
        sdk.setMotorTarget(targets);
        usleep(static_cast<useconds_t>(periodUs));
    }
}

bool waitMotorsOp(DriverSDK::DriverSDK& sdk,
                 int motorCount,
                 int periodUs,
                 int maxWaitTicks,
                 const std::vector<int>& waitIdx,
                 bool waitAllActive,
                 std::vector<DriverSDK::motorActualStruct>& actuals,
                 std::vector<DriverSDK::motorTargetStruct>& targets) {
    int waitTick = 0;
    int printEvery = 1;
    if (periodUs > 0) {
        printEvery = std::max(1, static_cast<int>(1000000 / periodUs));
    }

    while (!g_stop) {
        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets);

        for (int i = 0; i < motorCount; ++i) {
            targets[i].enabled = 0;
        }
        if (waitAllActive) {
            for (int i = 0; i < motorCount; ++i) {
                if (actuals[i].statusWord == 65535) {
                    continue;
                }
                targets[i].enabled = 1;
            }
        } else {
            for (int j = 0; j < (int)waitIdx.size(); ++j) {
                targets[waitIdx[j]].enabled = 1;
            }
        }
        sdk.setMotorTarget(targets);

        bool allOp = true;
        int activeCount = 0;
        int opCount = 0;
        if (waitAllActive) {
            for (int i = 0; i < motorCount; ++i) {
                if (actuals[i].statusWord == 65535) {
                    continue;
                }
                activeCount++;
                if (isOp(actuals[i].statusWord)) {
                    opCount++;
                } else {
                    allOp = false;
                }
            }
            if (activeCount == 0) {
                allOp = false;
            }
        } else {
            activeCount = (int)waitIdx.size();
            for (int j = 0; j < (int)waitIdx.size(); ++j) {
                const int idx = waitIdx[j];
                if (idx < 0 || idx >= motorCount) {
                    allOp = false;
                    continue;
                }
                if (isOp(actuals[idx].statusWord)) {
                    opCount++;
                } else {
                    allOp = false;
                }
            }
        }

        if (allOp) {
            return true;
        }

        if ((waitTick % printEvery) == 0) {
            std::cout << "[wait] tick=" << waitTick << " op=" << opCount << "/" << activeCount << "\n";
        }
        if (maxWaitTicks > 0 && waitTick >= maxWaitTicks) {
            std::cerr << "Timeout waiting for motors to reach OP.\n";
            if (waitAllActive) {
                for (int i = 0; i < motorCount; ++i) {
                    if (actuals[i].statusWord == 65535) {
                        continue;
                    }
                    if (!isOp(actuals[i].statusWord)) {
                        std::cerr << "  m" << i << " sw=0x" << std::hex << actuals[i].statusWord << std::dec << "\n";
                    }
                }
            } else {
                for (int j = 0; j < (int)waitIdx.size(); ++j) {
                    const int idx = waitIdx[j];
                    if (idx < 0 || idx >= motorCount) {
                        continue;
                    }
                    if (!isOp(actuals[idx].statusWord)) {
                        std::cerr << "  m" << idx << " sw=0x" << std::hex << actuals[idx].statusWord << std::dec << "\n";
                    }
                }
            }
            return false;
        }

        waitTick++;
        usleep(static_cast<useconds_t>(periodUs));
    }

    return false;
}

void sampleTrajectory(const std::vector<TrajFrame>& frames,
                      int dofArm,
                      double tSec,
                      int& seg,
                      std::vector<float>& qOut) {
    qOut.assign(dofArm, 0.0f);

    if (tSec <= frames.front().tSec) {
        qOut = frames.front().q;
        return;
    }
    if (tSec >= frames.back().tSec) {
        qOut = frames.back().q;
        return;
    }

    const int n = static_cast<int>(frames.size());
    while (seg + 1 < n && frames[seg + 1].tSec < tSec) {
        seg++;
    }
    const int i0 = seg;
    const int i1 = std::min(seg + 1, n - 1);
    const double t0 = frames[i0].tSec;
    const double t1 = frames[i1].tSec;
    const double denom = (t1 > t0) ? (t1 - t0) : 1.0;
    double a = (tSec - t0) / denom;
    if (a < 0.0) {
        a = 0.0;
    }
    if (a > 1.0) {
        a = 1.0;
    }
    const float s = static_cast<float>(a);

    for (int j = 0; j < dofArm; ++j) {
        const float q0 = frames[i0].q[j];
        const float q1 = frames[i1].q[j];
        qOut[j] = q0 + s * (q1 - q0);
    }
}
}

int main(int argc, char** argv) { 
    std::string configPath = "config.xml";
    int modeInt = 8;
    int periodUs = 1000;

    bool enableRight = false;
    bool enableLeft = false;

    std::string rightCsv;
    std::string leftCsv;

    std::string recordPath;
    std::string playPath;
    bool recordOnly = false;

    int blendMs = 1000;
    double waitOpTimeoutS = 30.0;
    int startDelayMs = 0;
    double playRate = 1.0;
    bool playLoop = false;
    bool waitAllMotors = false;

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
        if (arg == "--enable-right") {
            enableRight = true;
            continue;
        }
        if (arg == "--enable-left") {
            enableLeft = true;
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
        if (arg == "--record-right" && i + 1 < argc) {
            recordPath = argv[++i];
            continue;
        }
        if (arg == "--play" && i + 1 < argc) {
            playPath = argv[++i];
            continue;
        }
        if (arg == "--record-only") {
            recordOnly = true;
            continue;
        }
        if (arg == "--blend-ms" && i + 1 < argc) {
            blendMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--wait-op-timeout-s" && i + 1 < argc) {
            waitOpTimeoutS = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--start-delay-ms" && i + 1 < argc) {
            startDelayMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--play-rate" && i + 1 < argc) {
            playRate = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--play-loop") {
            playLoop = true;
            continue;
        }
        if (arg == "--wait-all-motors") {
            waitAllMotors = true;
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

    if (!recordPath.empty() && !playPath.empty()) {
        std::cerr << "--record-right and --play are mutually exclusive\n";
        return 1;
    }
    if (recordPath.empty() && playPath.empty()) {
        std::cerr << "Must provide either --record-right or --play\n";
        printUsage(argv[0]);
        return 1;
    }
    if (recordOnly && recordPath.empty()) {
        std::cerr << "--record-only requires --record-right\n";
        return 1;
    }

    if (periodUs <= 0) {
        periodUs = 1000;
    }
    if (blendMs < 0) {
        blendMs = 0;
    }
    if (waitOpTimeoutS < 0.0) {
        waitOpTimeoutS = 0.0;
    }
    if (startDelayMs < 0) {
        startDelayMs = 0;
    }
    if (playRate <= 0.0) {
        playRate = 1.0;
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
        DriverSDK::ConfigXML cfg(configPath.c_str());
        const std::vector<std::vector<int> > motorAlias = cfg.motorAlias();
        if (motorAlias.size() < 4) {
            std::cerr << "Invalid config.xml: missing limb mappings.\n";
            return 1;
        }
        leftArmAlias = motorAlias[2];
        rightArmAlias = motorAlias[3];
    }

    if (leftArmAlias.empty() || rightArmAlias.empty()) {
        std::cerr << "No arm motors found. Provide --left/--right or valid arm limbs in config.xml.\n";
        return 1;
    }
    if (leftArmAlias.size() != rightArmAlias.size()) {
        std::cerr << "Left/right arm DOF mismatch: left=" << leftArmAlias.size() << " right=" << rightArmAlias.size()
                  << "\n";
        return 1;
    }

    const int dofArm = static_cast<int>(rightArmAlias.size());

    std::vector<int> leftIdx(dofArm);
    std::vector<int> rightIdx(dofArm);
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

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);

    int maxWaitTicks = 0;
    if (waitOpTimeoutS > 0.0) {
        maxWaitTicks = static_cast<int>(waitOpTimeoutS * 1e6 / static_cast<double>(periodUs));
    }

    std::vector<int> waitIdx;
    if (!waitAllMotors) {
        if (enableLeft) {
            waitIdx.insert(waitIdx.end(), leftIdx.begin(), leftIdx.end());
        }
        if (enableRight) {
            waitIdx.insert(waitIdx.end(), rightIdx.begin(), rightIdx.end());
        }
        std::sort(waitIdx.begin(), waitIdx.end());
        waitIdx.erase(std::unique(waitIdx.begin(), waitIdx.end()), waitIdx.end());
        if (waitIdx.empty()) {
            std::cout << "[wait] no enabled motors selected; skipping OP wait\n";
        }
    }

    if (!waitIdx.empty() || waitAllMotors) {
        if (!waitMotorsOp(sdk, motorCount, periodUs, maxWaitTicks, waitIdx, waitAllMotors, actuals, targets)) {
            disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
            return 2;
        }
    }

    if (g_stop) {
        disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
        return 0;
    }

    if (startDelayMs > 0) {
        usleep(static_cast<useconds_t>(startDelayMs * 1000));
    }

    if (!recordPath.empty()) {
        std::ofstream out(recordPath.c_str(), std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << recordPath << "\n";
            disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
            return 1;
        }
        out << std::fixed << std::setprecision(6);
        out << "t";
        for (int j = 0; j < dofArm; ++j) {
            out << ",q" << j;
        }
        out << "\n";

        int tick = 0;
        while (!g_stop) {
            sdk.getMotorActual(actuals);

            if (!recordOnly) {
                fillHoldTargets(actuals, targets);

                for (int j = 0; j < dofArm; ++j) {
                    const int ri = rightIdx[j];
                    targets[ri].pos = actuals[ri].pos;
                    targets[ri].enabled = enableRight ? 1 : 0;
                }

                for (int j = 0; j < dofArm; ++j) {
                    const int ri = rightIdx[j];
                    const int li = leftIdx[j];
                    targets[li].pos = actuals[ri].pos;
                    targets[li].enabled = enableLeft ? 1 : 0;
                }

                sdk.setMotorTarget(targets);
            }

            const double tSec = static_cast<double>(tick) * static_cast<double>(periodUs) * 1e-6;
            out << tSec;
            for (int j = 0; j < dofArm; ++j) {
                out << "," << actuals[rightIdx[j]].pos;
            }
            out << "\n";

            tick++;
            usleep(static_cast<useconds_t>(periodUs));
        }

        disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
        return 0;
    }

    bool hasTime = false;
    std::vector<TrajFrame> frames;
    if (!loadTrajectoryCsv(playPath, dofArm, frames, hasTime)) {
        disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
        return 1;
    }

    if (!hasTime) {
        for (int i = 0; i < (int)frames.size(); ++i) {
            frames[i].tSec = static_cast<double>(i) * static_cast<double>(periodUs) * 1e-6;
        }
    }

    sdk.getMotorActual(actuals);
    std::vector<float> startLeft(dofArm, 0.0f);
    std::vector<float> startRight(dofArm, 0.0f);
    for (int j = 0; j < dofArm; ++j) {
        startLeft[j] = actuals[leftIdx[j]].pos;
        startRight[j] = actuals[rightIdx[j]].pos;
    }

    int blendTicks = 0;
    if (blendMs > 0) {
        blendTicks = std::max(1, (blendMs * 1000 + periodUs - 1) / periodUs);
    }

    for (int k = 0; k < blendTicks && !g_stop; ++k) {
        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets);

        const float s = static_cast<float>(k + 1) / static_cast<float>(blendTicks);
        for (int j = 0; j < dofArm; ++j) {
            const float q0 = frames[0].q[j];
            const float cmdR = startRight[j] + s * (q0 - startRight[j]);
            const float cmdL = startLeft[j] + s * (q0 - startLeft[j]);
            targets[rightIdx[j]].pos = cmdR;
            targets[leftIdx[j]].pos = cmdL;
            targets[rightIdx[j]].enabled = enableRight ? 1 : 0;
            targets[leftIdx[j]].enabled = enableLeft ? 1 : 0;
        }

        sdk.setMotorTarget(targets);
        usleep(static_cast<useconds_t>(periodUs));
    }

    int seg = 0;
    std::vector<float> qCmd;

    int tick = 0;
    while (!g_stop) {
        const double tSec = (static_cast<double>(tick) * static_cast<double>(periodUs) * 1e-6) * playRate;
        if (tSec > frames.back().tSec) {
            if (playLoop) {
                tick = 0;
                seg = 0;
                continue;
            }
            break;
        }

        sampleTrajectory(frames, dofArm, tSec, seg, qCmd);

        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets);

        for (int j = 0; j < dofArm; ++j) {
            targets[rightIdx[j]].pos = qCmd[j];
            targets[leftIdx[j]].pos = qCmd[j];
            targets[rightIdx[j]].enabled = enableRight ? 1 : 0;
            targets[leftIdx[j]].enabled = enableLeft ? 1 : 0;
        }

        sdk.setMotorTarget(targets);

        tick++;
        usleep(static_cast<useconds_t>(periodUs));
    }

    disableAllAndExit(sdk, motorCount, periodUs, actuals, targets);
    return 0;
}
