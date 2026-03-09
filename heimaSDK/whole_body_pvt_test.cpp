// Build (Linux; EtherCAT headers+lib in /usr/local; run in heimaSDK_copy/):
//   mkdir -p build && g++ -std=c++20 -O2 -pthread -DENABLE_RS232 whole_body_position_test.cpp heima_driver_sdk.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp rs232.cpp yesense_imu.cpp yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp rs485.cpp can.cpp -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm -o build/whole_body_position_test
//
// Usage (common commands):
//   # Help:
//   ./build/whole_body_position_test --help
//
//   # 1) Monitor all motors (no commands):
//   sudo ./build/whole_body_position_test --config config.xml --all --disable
//
//   # 2) Enable and hold current position for all motors:
//   sudo ./build/whole_body_position_test --config config.xml --all --enable
//
//   # 3) Select by limb (example: left arm):
//   sudo ./build/whole_body_position_test --config config.xml --limb 2 --enable
//
//   # 4) Select by aliases:
//   sudo ./build/whole_body_position_test --config config.xml --aliases 1,2,3,4,5,6 --enable
//
//   # 5) Go to fixed position (per-alias absolute position; continuous streaming):
//   sudo ./build/whole_body_position_test --config config.xml --enable --goto 5:0.5 --max-vel 0.2 --seconds -1
//   sudo ./build/whole_body_position_test --config config.xml --enable --goto 5:30deg,6:-15deg --max-vel 20deg/s --seconds -1
//
//   # 6) Back-and-forth (reciprocate) between two positions:
//   sudo ./build/whole_body_position_test --config config.xml --enable --recip 5:-0.3:0.3 --max-vel 0.2 --seconds -1
//   sudo ./build/whole_body_position_test --config config.xml --enable --deg --recip 5:-20:20 --max-vel 20 --recip-dwell-ms 200 --seconds -1
//
//   # 7) Sine test (selected group joint index):
//   sudo ./build/whole_body_position_test --config config.xml --legs --enable --joint 0 --amp 5deg --deg --freq 0.2
//
//   # 8) Sequential testing (test joints one by one):
//   sudo ./build/whole_body_position_test --config config.xml --all --enable --sequential --joint -1 --amp 5deg --deg --freq 0.2
//
//   # 9) Monitor motors + IMU data:
//   sudo ./build/whole_body_position_test --config config.xml --all --disable --imu
//
//   # 10) Calibrate selected motors (CountBias from current pose):
//   sudo ./build/whole_body_position_test --config config.xml --all --calibrate
//   sudo ./build/whole_body_position_test --config config.xml --limb 2 --calibrate-only
//
// Notes:
// - Motor selection: --all / --limb N / --legs / --arms / --waist / --neck / --aliases 1,2,3
// - Limb mapping: 0=left leg, 1=right leg, 2=left arm, 3=right arm, 4=waist, 5=neck
// - Safety: default is monitor-only; pass --enable to actually command motors.
// - Units: default rad; add --deg, or suffix per value: 30deg / 0.52rad / 20deg/s

#include "heima_driver_sdk.h"

#include "config_xml.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <unistd.h>

namespace {
volatile sig_atomic_t g_stop = 0;
float g_defaultKp = 0.0f;
float g_defaultKd = 0.0f;
float g_defaultTor = 0.0f;

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
        << "              [--goto a:p[,a:p...]] [--recip a:p1:p2[,a:p1:p2...]]\n"
        << "              [--max-vel rad_s 0.2] [--pos-eps rad 0.001] [--recip-dwell-ms 0]\n"
        << "              [--deg|--rad]\n"
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
        << "  --mode         CiA-402 mode (default: 5 PVT)\n"
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
        << "Alias Position Commands (position in rad by default, requires --enable):\n"
        << "  --goto a:p[,a:p...]          Move alias a to position p with velocity limiting\n"
        << "  --recip a:p1:p2[,a:p1:p2...] Back-and-forth between p1 and p2\n"
        << "  --max-vel     Max velocity for --goto/--recip in rad/s (default: 0.2)\n"
        << "  --pos-eps     Position tolerance for endpoint detection in rad (default: 0.001)\n"
        << "  --recip-dwell-ms  Dwell time at endpoints in ms (default: 0)\n"
        << "  --deg/--rad   Interpret plain numbers as degrees/radians (default: rad)\n"
        << "               You can also suffix per value: 30deg, 0.52rad, 20deg/s\n"
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

struct AliasPosCmd {
    int alias = 0;
    float pos = 0.0f;
};

struct AliasValueCmd {
    int alias = 0;
    float value = 0.0f;
};

struct AliasPosRangeCmd {
    int alias = 0;
    float posA = 0.0f;
    float posB = 0.0f;
};

bool endsWithIgnoreCase(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) {
        return false;
    }
    const size_t off = s.size() - suffix.size();
    for (size_t i = 0; i < suffix.size(); ++i) {
        const unsigned char a = static_cast<unsigned char>(s[off + i]);
        const unsigned char b = static_cast<unsigned char>(suffix[i]);
        if (std::tolower(a) != std::tolower(b)) {
            return false;
        }
    }
    return true;
}

bool parseDoubleStrict(const std::string& s, double& out, std::string& err) {
    err.clear();
    size_t idx = 0;
    try {
        out = std::stod(s, &idx);
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
    while (idx < s.size() && std::isspace(static_cast<unsigned char>(s[idx]))) {
        ++idx;
    }
    if (idx != s.size()) {
        err = "trailing chars";
        return false;
    }
    return true;
}

bool parseAngleRad(const std::string& s, bool defaultDeg, double& outRad, std::string& err) {
    err.clear();
    std::string t = s;
    while (!t.empty() && std::isspace(static_cast<unsigned char>(t.front()))) {
        t.erase(t.begin());
    }
    while (!t.empty() && std::isspace(static_cast<unsigned char>(t.back()))) {
        t.pop_back();
    }
    if (t.empty()) {
        err = "empty angle";
        return false;
    }

    enum class Unit { Rad, Deg };
    Unit unit = defaultDeg ? Unit::Deg : Unit::Rad;
    std::string num = t;

    if (endsWithIgnoreCase(t, "deg")) {
        unit = Unit::Deg;
        num = t.substr(0, t.size() - 3);
    } else if (endsWithIgnoreCase(t, "rad")) {
        unit = Unit::Rad;
        num = t.substr(0, t.size() - 3);
    }

    double v = 0.0;
    if (!parseDoubleStrict(num, v, err)) {
        err = "invalid angle: " + t;
        return false;
    }

    static const double pi = std::acos(-1.0);
    outRad = (unit == Unit::Deg) ? (v * pi / 180.0) : v;
    return true;
}

bool parseAngleRateRadPerSec(const std::string& s, bool defaultDeg, double& outRadPerSec, std::string& err) {
    err.clear();
    std::string t = s;
    while (!t.empty() && std::isspace(static_cast<unsigned char>(t.front()))) {
        t.erase(t.begin());
    }
    while (!t.empty() && std::isspace(static_cast<unsigned char>(t.back()))) {
        t.pop_back();
    }
    if (t.empty()) {
        err = "empty rate";
        return false;
    }

    enum class Unit { Rad, Deg };
    Unit unit = defaultDeg ? Unit::Deg : Unit::Rad;
    std::string num = t;

    const auto takeSuffix = [&](const std::string& suffix, Unit u) {
        if (endsWithIgnoreCase(t, suffix)) {
            unit = u;
            num = t.substr(0, t.size() - suffix.size());
            return true;
        }
        return false;
    };

    if (!takeSuffix("deg/s", Unit::Deg) &&
        !takeSuffix("deg_s", Unit::Deg) &&
        !takeSuffix("dps", Unit::Deg) &&
        !takeSuffix("rad/s", Unit::Rad) &&
        !takeSuffix("rad_s", Unit::Rad) &&
        !takeSuffix("rps", Unit::Rad) &&
        !takeSuffix("deg", Unit::Deg) &&
        !takeSuffix("rad", Unit::Rad)) {
    }

    double v = 0.0;
    if (!parseDoubleStrict(num, v, err)) {
        err = "invalid rate: " + t;
        return false;
    }

    static const double pi = std::acos(-1.0);
    outRadPerSec = (unit == Unit::Deg) ? (v * pi / 180.0) : v;
    return true;
}

bool parseAliasPosCmds(const std::string& spec, bool defaultDeg, std::vector<AliasPosCmd>& out, std::string& err) {
    out.clear();
    err.clear();
    size_t start = 0;
    while (start < spec.size()) {
        size_t end = spec.find(',', start);
        std::string token = (end == std::string::npos) ? spec.substr(start) : spec.substr(start, end - start);
        if (!token.empty()) {
            const size_t sep = token.find(':');
            if (sep == std::string::npos) {
                err = "Expected alias:pos in --goto, got: " + token;
                return false;
            }
            const std::string aStr = token.substr(0, sep);
            const std::string pStr = token.substr(sep + 1);
            if (aStr.empty() || pStr.empty()) {
                err = "Invalid alias:pos in --goto, got: " + token;
                return false;
            }
            AliasPosCmd cmd;
            cmd.alias = std::stoi(aStr);
            double posRad = 0.0;
            if (!parseAngleRad(pStr, defaultDeg, posRad, err)) {
                err = "Invalid --goto pos for alias " + aStr + ": " + pStr;
                return false;
            }
            cmd.pos = static_cast<float>(posRad);
            out.push_back(cmd);
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

bool parseAliasValueCmds(const std::string& spec, std::vector<AliasValueCmd>& out, std::string& err) {
    out.clear();
    err.clear();
    size_t start = 0;
    while (start < spec.size()) {
        size_t end = spec.find(',', start);
        std::string token = (end == std::string::npos) ? spec.substr(start) : spec.substr(start, end - start);
        if (!token.empty()) {
            const size_t sep = token.find(':');
            if (sep == std::string::npos) {
                err = "Expected alias:value, got: " + token;
                return false;
            }
            const std::string aStr = token.substr(0, sep);
            const std::string vStr = token.substr(sep + 1);
            if (aStr.empty() || vStr.empty()) {
                err = "Invalid alias:value, got: " + token;
                return false;
            }
            AliasValueCmd cmd;
            cmd.alias = std::stoi(aStr);
            cmd.value = std::stof(vStr);
            out.push_back(cmd);
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

bool parseAliasPosRangeCmds(const std::string& spec, bool defaultDeg, std::vector<AliasPosRangeCmd>& out, std::string& err) {
    out.clear();
    err.clear();
    size_t start = 0;
    while (start < spec.size()) {
        size_t end = spec.find(',', start);
        std::string token = (end == std::string::npos) ? spec.substr(start) : spec.substr(start, end - start);
        if (!token.empty()) {
            const size_t sep1 = token.find(':');
            const size_t sep2 = (sep1 == std::string::npos) ? std::string::npos : token.find(':', sep1 + 1);
            if (sep1 == std::string::npos || sep2 == std::string::npos) {
                err = "Expected alias:pos1:pos2 in --recip, got: " + token;
                return false;
            }
            const std::string aStr = token.substr(0, sep1);
            const std::string p1Str = token.substr(sep1 + 1, sep2 - sep1 - 1);
            const std::string p2Str = token.substr(sep2 + 1);
            if (aStr.empty() || p1Str.empty() || p2Str.empty()) {
                err = "Invalid alias:pos1:pos2 in --recip, got: " + token;
                return false;
            }
            AliasPosRangeCmd cmd;
            cmd.alias = std::stoi(aStr);
            double p1Rad = 0.0;
            if (!parseAngleRad(p1Str, defaultDeg, p1Rad, err)) {
                err = "Invalid --recip pos1 for alias " + aStr + ": " + p1Str;
                return false;
            }
            double p2Rad = 0.0;
            if (!parseAngleRad(p2Str, defaultDeg, p2Rad, err)) {
                err = "Invalid --recip pos2 for alias " + aStr + ": " + p2Str;
                return false;
            }
            cmd.posA = static_cast<float>(p1Rad);
            cmd.posB = static_cast<float>(p2Rad);
            out.push_back(cmd);
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

float stepToward(float current, float target, float maxStep) {
    if (maxStep <= 0.0f) {
        return current;
    }
    const float err = target - current;
    if (std::fabs(err) <= maxStep) {
        return target;
    }
    return current + ((err > 0.0f) ? maxStep : -maxStep);
}

struct RecipState {
    float posA = 0.0f;
    float posB = 0.0f;
    bool towardB = false;
    int dwellTicks = 0;
    int dwellRemaining = 0;
};

void fillHoldTargets(const std::vector<DriverSDK::motorActualStruct>& actuals,
                     std::vector<DriverSDK::motorTargetStruct>& targets,
                     const std::vector<float>& motorKp,
                     const std::vector<float>& motorKd,
                     const std::vector<float>& motorTor) {
    const int motorCount = static_cast<int>(targets.size());
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = actuals[i].pos;
        targets[i].vel = 0.0f;
        targets[i].tor = motorTor[i];
        targets[i].kp = motorKp[i];
        targets[i].kd = motorKd[i];
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
                  << " mt_tmp=" << actuals[idx].temp
                  << " dr_tmp=" << actuals[idx].driveTemp
                  << " vol=" << actuals[idx].voltage
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
                      std::vector<DriverSDK::motorTargetStruct>& targets,
                      const std::vector<float>& motorKp,
                      const std::vector<float>& motorKd,
                      const std::vector<float>& motorTor) {
    sdk.getMotorActual(actuals);
    fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);
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

    int modeInt = 5;
    int periodUs = 1000;
    double seconds = 10.0;
    bool enable = false;
    bool followActual = false;
    int settleMs = 0;
    int joint = -1;
    bool jointSet = false;
    double amp = 0.0;
    bool ampSet = false;
    std::string ampSpec;
    double freq = 0.0;
    bool freqSet = false;
    bool sequential = false;
    int sequentialDelayMs = 1000;
    int printMs = 200;
    int maxCurrentU16 = -1;
    std::string gotoSpec;
    std::string recipSpec;
    double maxVel = 0.2;
    double posEps = 1e-3;
    bool maxVelSet = false;
    std::string maxVelSpec;
    bool posEpsSet = false;
    std::string posEpsSpec;
    int recipDwellMs = 0;
    bool angleDefaultDeg = false;

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
            ampSpec = argv[++i];
            ampSet = true;
            continue;
        }
        if (arg == "--freq" && i + 1 < argc) {
            freq = std::stod(argv[++i]);
            freqSet = true;
            continue;
        }
        if (arg == "--goto" && i + 1 < argc) {
            gotoSpec = argv[++i];
            continue;
        }
        if (arg == "--recip" && i + 1 < argc) {
            recipSpec = argv[++i];
            continue;
        }
        if (arg == "--max-vel" && i + 1 < argc) {
            maxVelSpec = argv[++i];
            maxVelSet = true;
            continue;
        }
        if (arg == "--pos-eps" && i + 1 < argc) {
            posEpsSpec = argv[++i];
            posEpsSet = true;
            continue;
        }
        if (arg == "--recip-dwell-ms" && i + 1 < argc) {
            recipDwellMs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--deg") {
            angleDefaultDeg = true;
            continue;
        }
        if (arg == "--rad") {
            angleDefaultDeg = false;
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
    if (recipDwellMs < 0) {
        recipDwellMs = 0;
    }

    {
        std::string err;
        if (maxVelSet) {
            double velRadPerSec = 0.0;
            if (!parseAngleRateRadPerSec(maxVelSpec, angleDefaultDeg, velRadPerSec, err)) {
                std::cerr << err << "\n";
                return 1;
            }
            maxVel = velRadPerSec;
        }
        if (posEpsSet) {
            double epsRad = 0.0;
            if (!parseAngleRad(posEpsSpec, angleDefaultDeg, epsRad, err)) {
                std::cerr << err << "\n";
                return 1;
            }
            posEps = epsRad;
        }
    }

    if (maxVel < 0.0) {
        maxVel = -maxVel;
    }
    if (posEps < 0.0) {
        posEps = -posEps;
    }

    std::vector<AliasPosCmd> gotoCmds;
    std::vector<AliasPosRangeCmd> recipCmds;
    std::vector<int> autoSelectAliases;
    {
        std::string err;
        if (!gotoSpec.empty()) {
            if (!parseAliasPosCmds(gotoSpec, angleDefaultDeg, gotoCmds, err)) {
                std::cerr << err << "\n";
                return 1;
            }
            for (const auto& c : gotoCmds) {
                autoSelectAliases.push_back(c.alias);
            }
        }
        if (!recipSpec.empty()) {
            if (!parseAliasPosRangeCmds(recipSpec, angleDefaultDeg, recipCmds, err)) {
                std::cerr << err << "\n";
                return 1;
            }
            for (const auto& c : recipCmds) {
                autoSelectAliases.push_back(c.alias);
            }
        }
    }

    if (jointSet && joint < -1) {
        std::cerr << "Invalid --joint: " << joint << " (must be >=0 or -1)\n";
        return 1;
    }
    if (jointSet) {
        std::string err;
        if (ampSet) {
            double ampRad = 0.0;
            if (!parseAngleRad(ampSpec, angleDefaultDeg, ampRad, err)) {
                std::cerr << err << "\n";
                return 1;
            }
            amp = ampRad;
        }
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
    if ((!gotoCmds.empty() || !recipCmds.empty()) && !enable) {
        std::cout << "--goto/--recip implies --enable\n";
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
        for (int a : autoSelectAliases) {
            selectedAlias.push_back(a);
        }
    } else {
        const bool hasExplicitSelection =
            selectAll || limb >= 0 || selectLegs || selectArms || selectWaist || selectNeck;
        if (!hasExplicitSelection) {
            if (autoSelectAliases.empty()) {
                std::cerr << "No motors selected. Use --all, --limb, --legs, --arms, --waist, --neck, or --aliases.\n";
                printUsage(argv[0]);
                return 1;
            }
            selectedAlias = autoSelectAliases;
            selectionDesc = "aliases(from goto/recip)";
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
            for (int a : autoSelectAliases) {
                selectedAlias.push_back(a);
            }
        }
    }

    // ------------------------------------------------------------------------
    // PER-MOTOR PVT PARAMETERS
    // You can set the Kp, Kd, and Torque parameters for each motor here.
    // The index in these arrays matches the output of (Alias - 1).
    // For example, the first element is for Alias 1, the second for Alias 2, etc.
    // If you don't want to use these hardcoded arrays, you can fall back to g_defaultKp.
    // ------------------------------------------------------------------------
    const float kp_array[] = {
        0.0f,  // Alias 1
        0.0f,  // Alias 2
        0.0f,  // Alias 3
        0.0f,  // Alias 4
        0.0f,  // Alias 5
        0.0f,  // Alias 6
        0.0f,  // Alias 7
        0.0f,  // Alias 8
        0.0f,  // Alias 9
        0.0f,  // Alias 10
        0.0f,  // Alias 11
        0.0f,  // Alias 12
        0.0f,  // Alias 13
        0.0f,  // Alias 14
        0.0f   // Alias 15
    };
    
    const float kd_array[] = {
        0.0f,  // Alias 1
        0.0f,  // Alias 2
        0.0f,  // Alias 3
        0.0f,  // Alias 4
        0.0f,  // Alias 5
        0.0f,  // Alias 6
        0.0f,  // Alias 7
        0.0f,  // Alias 8
        0.0f,  // Alias 9
        0.0f,  // Alias 10
        0.0f,  // Alias 11
        0.0f,  // Alias 12
        0.0f,  // Alias 13
        0.0f,  // Alias 14
        0.0f   // Alias 15
    };
    
    const float tor_array[] = {
        0.0f,  // Alias 1
        0.0f,  // Alias 2
        0.0f,  // Alias 3
        0.0f,  // Alias 4
        0.0f,  // Alias 5
        0.0f,  // Alias 6
        0.0f,  // Alias 7
        0.0f,  // Alias 8
        0.0f,  // Alias 9
        0.0f,  // Alias 10
        0.0f,  // Alias 11
        0.0f,  // Alias 12
        0.0f,  // Alias 13
        0.0f,  // Alias 14
        0.0f   // Alias 15
    };

    std::vector<float> motorKp(motorCount, g_defaultKp);
    std::vector<float> motorKd(motorCount, g_defaultKd);
    std::vector<float> motorTor(motorCount, g_defaultTor);

    for (int i = 0; i < motorCount; ++i) {
        if (i < static_cast<int>(sizeof(kp_array) / sizeof(kp_array[0]))) {
            motorKp[i] = kp_array[i];
        }
        if (i < static_cast<int>(sizeof(kd_array) / sizeof(kd_array[0]))) {
            motorKd[i] = kd_array[i];
        }
        if (i < static_cast<int>(sizeof(tor_array) / sizeof(tor_array[0]))) {
            motorTor[i] = tor_array[i];
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
    if (!gotoCmds.empty()) {
        std::cout << "Goto: " << gotoSpec << " max_vel=" << maxVel << " rad/s\n";
    }
    if (!recipCmds.empty()) {
        std::cout << "Recip: " << recipSpec << " max_vel=" << maxVel << " rad/s dwell_ms=" << recipDwellMs << "\n";
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

    std::vector<int> hasGoto(static_cast<size_t>(motorIdx.size()), 0);
    std::vector<float> gotoTarget(static_cast<size_t>(motorIdx.size()), 0.0f);
    std::vector<int> hasRecip(static_cast<size_t>(motorIdx.size()), 0);
    std::vector<RecipState> recipState(static_cast<size_t>(motorIdx.size()));

    if (!gotoCmds.empty() || !recipCmds.empty()) {
        std::vector<int> aliasToSelectedIndex(static_cast<size_t>(motorCount + 1), -1);
        for (int j = 0; j < static_cast<int>(motorAliasForPrint.size()); ++j) {
            const int a = motorAliasForPrint[j];
            if (a >= 0 && a <= motorCount) {
                aliasToSelectedIndex[a] = j;
            }
        }

        for (const auto& cmd : gotoCmds) {
            if (cmd.alias <= 0 || cmd.alias > motorCount) {
                std::cerr << "Invalid alias in --goto: " << cmd.alias << " (must be 1-" << motorCount << ")\n";
                return 1;
            }
            const int j = aliasToSelectedIndex[cmd.alias];
            if (j < 0) {
                std::cerr << "Alias in --goto not selected: " << cmd.alias << "\n";
                return 1;
            }
            if (hasGoto[j]) {
                std::cerr << "Duplicate alias in --goto: " << cmd.alias << "\n";
                return 1;
            }
            hasGoto[j] = 1;
            gotoTarget[j] = cmd.pos;
        }

        const int dwellTicks = std::max(0, (recipDwellMs * 1000 + periodUs - 1) / periodUs);
        for (const auto& cmd : recipCmds) {
            if (cmd.alias <= 0 || cmd.alias > motorCount) {
                std::cerr << "Invalid alias in --recip: " << cmd.alias << " (must be 1-" << motorCount << ")\n";
                return 1;
            }
            const int j = aliasToSelectedIndex[cmd.alias];
            if (j < 0) {
                std::cerr << "Alias in --recip not selected: " << cmd.alias << "\n";
                return 1;
            }
            if (hasRecip[j]) {
                std::cerr << "Duplicate alias in --recip: " << cmd.alias << "\n";
                return 1;
            }
            if (hasGoto[j]) {
                std::cerr << "Alias " << cmd.alias << " specified in both --goto and --recip.\n";
                return 1;
            }
            if (cmd.posA == cmd.posB) {
                std::cerr << "Invalid --recip for alias " << cmd.alias << ": pos1 == pos2\n";
                return 1;
            }
            hasRecip[j] = 1;
            recipState[j].posA = cmd.posA;
            recipState[j].posB = cmd.posB;
            recipState[j].towardB = false;
            recipState[j].dwellTicks = dwellTicks;
            recipState[j].dwellRemaining = 0;
        }
    }

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

    const int maxWaitTicks = (int)(100.0 * 1e6 / static_cast<double>(periodUs));
    int waitTick = 0;
    while (!g_stop) {
        sdk.getMotorActual(actuals);
        fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);
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
        disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets, motorKp, motorKd, motorTor);
        return 0;
    }

    if (calibrate) {
        std::cout << "\n=== Calibration: place robot at zero pose and keep still ===\n";
        const int maxWaitTicks = (int)(10.0 * 1e6 / static_cast<double>(periodUs));
        int waitTick = 0;
        bool allOp = false;
        while (!g_stop) {
            sdk.getMotorActual(actuals);
            fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);
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
            disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets, motorKp, motorKd, motorTor);
            return 0;
        }
    }

    sdk.getMotorActual(actuals);

    std::vector<float> basePos(motorIdx.size(), 0.0f);
    for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
        basePos[j] = actuals[motorIdx[j]].pos;
    }
    std::vector<float> cmdPos = basePos;

    const float maxStep = static_cast<float>(maxVel * static_cast<double>(periodUs) * 1e-6);
    const float posEpsF = static_cast<float>(posEps);

    if (!followActual && settleMs > 0) {
        const int settleTicks = std::max(1, (settleMs * 1000 + periodUs - 1) / periodUs);
        for (int s = 0; s < settleTicks && !g_stop; ++s) {
            sdk.getMotorActual(actuals);
            fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);
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
        cmdPos = basePos;
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
                fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);
                for (int idx : motorIdx) {
                    targets[idx].enabled = 1;
                }
                const int activeIdx = motorIdx[testIdx];
                targets[activeIdx].pos = actuals[activeIdx].pos;
                sdk.setMotorTarget(targets);
                usleep(1000);
            }
            cmdPos[testIdx] = actuals[motorIdx[testIdx]].pos;

            int seqTick = 0;
            const double maxSeqTicks = (seconds < 0.0) ? -1.0 : (seconds * 1e6 / static_cast<double>(periodUs));
            while (!g_stop) {
                sdk.getMotorActual(actuals);
                if (enableImu) {
                    sdk.getIMU(imu);
                }
                fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);

                for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
                    const int idx = motorIdx[j];
                    targets[idx].enabled = 1;
                    if (j == testIdx) {
                        float desired = basePos[testIdx];
                        if (amp > 0.0 && freq > 0.0) {
                            const double t = static_cast<double>(seqTick) * static_cast<double>(periodUs) * 1e-6;
                            desired = static_cast<float>(basePos[testIdx] + amp * std::sin(twoPi * freq * t));
                        }
                        if (hasRecip[testIdx]) {
                            RecipState& st = recipState[testIdx];
                            const float endpoint = st.towardB ? st.posB : st.posA;
                            if (st.dwellRemaining > 0) {
                                cmdPos[testIdx] = endpoint;
                                desired = cmdPos[testIdx];
                                if (--st.dwellRemaining == 0) {
                                    st.towardB = !st.towardB;
                                }
                            } else {
                                const bool reachedCmd = (cmdPos[testIdx] == endpoint);
                                const bool reachedActual = (std::fabs(endpoint - actuals[idx].pos) <= posEpsF);
                                if (reachedCmd || reachedActual) {
                                    if (st.dwellTicks > 0) {
                                        cmdPos[testIdx] = endpoint;
                                        desired = cmdPos[testIdx];
                                        st.dwellRemaining = st.dwellTicks;
                                    } else {
                                        cmdPos[testIdx] = endpoint;
                                        desired = cmdPos[testIdx];
                                        st.towardB = !st.towardB;
                                    }
                                } else {
                                    cmdPos[testIdx] = stepToward(cmdPos[testIdx], endpoint, maxStep);
                                    desired = cmdPos[testIdx];
                                }
                            }
                        } else if (hasGoto[testIdx]) {
                            cmdPos[testIdx] = stepToward(cmdPos[testIdx], gotoTarget[testIdx], maxStep);
                            desired = cmdPos[testIdx];
                        }
                        targets[idx].pos = desired;
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
            fillHoldTargets(actuals, targets, motorKp, motorKd, motorTor);

            for (int j = 0; j < static_cast<int>(motorIdx.size()); ++j) {
                const int idx = motorIdx[j];
                targets[idx].enabled = 1;
                float desired = targets[idx].pos;
                if (hasRecip[j]) {
                    RecipState& st = recipState[j];
                    const float endpoint = st.towardB ? st.posB : st.posA;
                    if (st.dwellRemaining > 0) {
                        cmdPos[j] = endpoint;
                        desired = cmdPos[j];
                        if (--st.dwellRemaining == 0) {
                            st.towardB = !st.towardB;
                        }
                    } else {
                        const bool reachedCmd = (cmdPos[j] == endpoint);
                        const bool reachedActual = (std::fabs(endpoint - actuals[idx].pos) <= posEpsF);
                        if (reachedCmd || reachedActual) {
                            if (st.dwellTicks > 0) {
                                cmdPos[j] = endpoint;
                                desired = cmdPos[j];
                                st.dwellRemaining = st.dwellTicks;
                            } else {
                                cmdPos[j] = endpoint;
                                desired = cmdPos[j];
                                st.towardB = !st.towardB;
                            }
                        } else {
                            cmdPos[j] = stepToward(cmdPos[j], endpoint, maxStep);
                            desired = cmdPos[j];
                        }
                    }
                } else if (hasGoto[j]) {
                    cmdPos[j] = stepToward(cmdPos[j], gotoTarget[j], maxStep);
                    desired = cmdPos[j];
                } else if (!followActual) {
                    desired = basePos[j];
                    if (jointSet && (joint == -1 || j == joint) && amp > 0.0 && freq > 0.0) {
                        const double t = static_cast<double>(tick) * static_cast<double>(periodUs) * 1e-6;
                        desired = static_cast<float>(basePos[j] + amp * std::sin(twoPi * freq * t));
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

    disableAllAndExit(sdk, motorIdx, periodUs, actuals, targets, motorKp, motorKd, motorTor);
    return 0;
}



///sudo ./build/whole_body_position_test --config config.xml --enable --recip 4:1.39:0,10:1.4:0,3:0.8:-1.1,9:1:-1.3 --max-vel 1 --seconds -1 --all
///sudo ./build/whole_body_position_test --config config.xml --enable --recip 4:1.39:0,10:1.4:0 --max-vel 1 --seconds -1 --all
///sudo ./build/whole_body_position_test --config config.xml --enable --recip 3:0.8:-1.1,9:1:-1.3 --max-vel 1 --seconds -1 --all
