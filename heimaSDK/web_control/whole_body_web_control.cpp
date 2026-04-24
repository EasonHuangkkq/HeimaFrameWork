// Build (Linux; EtherCAT headers+lib in /usr/local; run in heimaSDK_copy/):
// g++ -std=c++20 -O2 -pthread -DENABLE_RS232 whole_body_pvt_simple.cpp heima_driver_sdk.cpp ecat.cpp common.cpp config_xml.cpp tinyxml2.cpp rs232.cpp yesense_imu.cpp yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp rs485.cpp can.cpp -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm -o build/whole_body_pvt_simple

#include "heima_driver_sdk.h"
#include "config_xml.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <map>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <cstring>

namespace {
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) { g_stop = 1; }

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

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [--config config.xml] [--all] [--limb N]\n"
              << "       [--aliases 1,2,3] [--goto 1:0.5,2:-0.5] [--max-vel 0.5]\n"
              << "       [--kp 12.0] [--kd 0.2] [--period-us 1000] [--enable]\n\n"
              << "Limb indices: 0=LL, 1=RL, 2=LA, 3=RA, 4=Waist, 5=Neck\n";
}

float stepToward(float current, float target, float maxStep) {
    if (maxStep <= 0.0f) return current;
    const float err = target - current;
    if (std::fabs(err) <= maxStep) return target;
    return current + ((err > 0.0f) ? maxStep : -maxStep);
}

// 直观呈现的 OP 状态检查判断逻辑
bool isOp(unsigned short statusWord) {
    return (statusWord & 0x007f) == 0x0037;
}

} // namespace

int main(int argc, char** argv) {
    std::string configPath = "../../config.xml";
    bool selectAll = false;
    int limb = -1;
    std::string aliasesCsv = "";
    std::string gotoCsv = "";
    double maxVel = 0.2; // rad/s
    int periodUs = 1000;
    bool enable = false;

    // ------------------------------------------------------------------------
    // 【独立参数配置区】每个电机的 PVT 参数，数组索引 i 对应 Alias (i+1)
    // 依据总线顺序（13个电机）：先右腿(1~6)，再左腿(7~12)，最后腰部(13)
    // ------------------------------------------------------------------------
    const float kp_array[] = {
        // --- 右腿 (Right Leg, Alias 1~6) ---
        100.0f,  // 1: 髋 roll
        100.0f,  // 2: 髋 yaw
        200.0f,  // 3: 髋 pitch
        200.0f,  // 4: 膝盖 pitch
        12.0f, // 5: 踝 pitch
        12.0f, // 6: 踝 roll
        // --- 左腿 (Left Leg, Alias 7~12) ---
        100.0f,  // 7: 髋 roll
        100.0f,  // 8: 髋 yaw
        200.0f,  // 9: 髋 pitch
        200.0f,  // 10: 膝盖 pitch
        12.0f, // 11: 踝 pitch
        12.0f, // 12: 踝 roll
        // --- 腰部 (Waist, Alias 13) ---
        60.0f   // 13: 腰 yaw
    };
    
    const float kd_array[] = {
        // 右腿 1~6
        2.0f, 2.0f, 2.0f, 4.0f, 0.5f, 0.5f,
        // 左腿 7~12
        2.0f, 2.0f, 2.0f, 4.0f, 0.5f, 0.5f,
        // 腰部 13
        2.0f
    };
    
    const float tor_array[] = {
        // 右腿 1~6
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        // 左腿 7~12
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        // 腰部 13
        0.0f
    };

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { printUsage(argv[0]); return 0; }
        else if (arg == "--config" && i+1 < argc) configPath = argv[++i];
        else if (arg == "--all") selectAll = true;
        else if (arg == "--limb" && i+1 < argc) limb = std::stoi(argv[++i]);
        else if (arg == "--aliases" && i+1 < argc) aliasesCsv = argv[++i];
        else if (arg == "--goto" && i+1 < argc) gotoCsv = argv[++i];
        else if (arg == "--max-vel" && i+1 < argc) maxVel = std::stod(argv[++i]);
        else if (arg == "--period-us" && i+1 < argc) periodUs = std::stoi(argv[++i]);
        else if (arg == "--enable") enable = true;
        else { std::cerr << "Unknown arg: " << arg << "\n"; return 1; }
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());

    const int motorCount = sdk.getTotalMotorNr();
    if (motorCount <= 0) { std::cerr << "No motors in XML!\n"; return 1; }

    std::vector<float> motorKp(motorCount, 0.0f);
    std::vector<float> motorKd(motorCount, 0.0f);
    std::vector<float> motorTor(motorCount, 0.0f);
    for (int i = 0; i < motorCount; ++i) {
        if (i < sizeof(kp_array) / sizeof(kp_array[0])) motorKp[i] = kp_array[i];
        if (i < sizeof(kd_array) / sizeof(kd_array[0])) motorKd[i] = kd_array[i];
        if (i < sizeof(tor_array) / sizeof(tor_array[0])) motorTor[i] = tor_array[i];
    }

    std::vector<int> selectedAlias;
    if (selectAll) {
        for (int i=1; i<=motorCount; i++) selectedAlias.push_back(i);
    } else if (limb >= 0 && limb < 6) {
        DriverSDK::ConfigXML cfg(configPath.c_str());
        auto mAlias = cfg.motorAlias();
        if (limb < mAlias.size()) {
            for (int a : mAlias[limb]) if (a > 0) selectedAlias.push_back(a);
        }
    } else if (!aliasesCsv.empty()) {
        size_t start = 0;
        while (start < aliasesCsv.size()) {
            size_t end = aliasesCsv.find(',', start);
            std::string token = (end == std::string::npos) ? aliasesCsv.substr(start) : aliasesCsv.substr(start, end - start);
            if (!token.empty()) selectedAlias.push_back(std::stoi(token));
            if (end == std::string::npos) break;
            start = end + 1;
        }
    }

    std::map<int, float> gotoTargets;
    if (!gotoCsv.empty()) {
        size_t start = 0;
        while (start < gotoCsv.size()) {
            size_t end = gotoCsv.find(',', start);
            std::string token = (end == std::string::npos) ? gotoCsv.substr(start) : gotoCsv.substr(start, end - start);
            if (!token.empty()) {
                size_t sep = token.find(':');
                if (sep != std::string::npos) {
                    int a = std::stoi(token.substr(0, sep));
                    float p = std::stof(token.substr(sep + 1));
                    gotoTargets[a] = p;
                    // Auto select the motor if someone uses --goto without explicit selection
                    if (std::find(selectedAlias.begin(), selectedAlias.end(), a) == selectedAlias.end()) {
                        selectedAlias.push_back(a);
                    }
                }
            }
            if (end == std::string::npos) break;
            start = end + 1;
        }
    }

    if (selectedAlias.empty()) {
        std::cerr << "No motors selected! Use --all, --limb, or --aliases.\n";
        return 1;
    }

    std::sort(selectedAlias.begin(), selectedAlias.end());
    selectedAlias.erase(std::unique(selectedAlias.begin(), selectedAlias.end()), selectedAlias.end());

    std::vector<int> motorIdx;
    for (int a : selectedAlias) {
        if (a > 0 && a <= motorCount) {
            motorIdx.push_back(a - 1);
        }
    }

    if (motorIdx.empty()) { std::cerr << "No valid motors matched selection.\n"; return 1; }

    std::cout << "Starting PVT / Pose Test. Motors selected: " << motorIdx.size() << "\n";
    if (!enable) std::cout << "WARNING: --enable not specified. Read-only mode.\n";

    // 绑定轻量级非阻塞 UDP 监听器用于动态调参
    int udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSock < 0) {
        std::cerr << "UDP Socket creation failed.\n";
    } else {
        struct sockaddr_in servaddr;
        std::memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(8888);
        if (bind(udpSock, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            std::cerr << "UDP Bind failed (port 8888). Is another instance running?\n";
            udpSock = -1;
        } else {
            int flags = fcntl(udpSock, F_GETFL, 0);
            fcntl(udpSock, F_SETFL, flags | O_NONBLOCK);
            std::cout << "UDP Listener started on port 8888. Ready for dynamic commands.\n";
        }
    }

    // Set PVT Mode (5)
    std::vector<char> modes(motorCount, 5);
    sdk.setMode(modes);

    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);

    if (!enable) {
        std::cout << "Monitoring only...\n";
        int tick = 0;
        while (!g_stop) {
            int ret = sdk.getMotorActual(actuals);
            if (++tick % 200 == 0) {
                std::cout << "--- Tick " << tick << (ret < 0 ? " [EtherCAT 通讯异常! 以下可能为死数据]" : "") << " ---\n";
                for (int idx : motorIdx) {
                    std::cout << " Alias=" << (idx+1) 
                              << " Pos=" << actuals[idx].pos 
                              << " Err=" << getErrorString(actuals[idx].errorCode) 
                              << " SW=0x" << std::hex << (actuals[idx].statusWord & 0xFFFF) << std::dec << "\n";
                }
            }
            usleep(periodUs);
        }
        return 0;
    }

    // ENABLE MODE: Wait for OP
    std::cout << "Waiting for OP...\n";
    int waitTick = 0;
    while (!g_stop) {
        int ret = sdk.getMotorActual(actuals);
        bool allOp = true;
        for (int i = 0; i < motorCount; i++) {
            targets[i].enabled = (std::find(motorIdx.begin(), motorIdx.end(), i) != motorIdx.end()) ? 1 : 0;
            targets[i].pos = actuals[i].pos; // Hold current position!
            targets[i].vel = 0;
            targets[i].tor = motorTor[i];
            targets[i].kp = motorKp[i];
            targets[i].kd = motorKd[i];
            if (targets[i].enabled) {
                if (!isOp(actuals[i].statusWord) || ret < 0) {
                    allOp = false; // 必须达到 OP，并且 EtherCAT 通讯不能报错
                }
            }
        }
        sdk.setMotorTarget(targets);
        if (allOp) break;
        if (++waitTick % 200 == 0) std::cout << "Still waiting for OP...\n";
        usleep(periodUs);
    }

    if (g_stop) return 0;
    std::cout << "All selected motors in OP. Proceeding to PVT holding/movement.\n";

    std::vector<float> currentPos(motorCount, 0.0f);
    for (int idx : motorIdx) currentPos[idx] = actuals[idx].pos;

    float maxStep = maxVel * (periodUs * 1e-6);
    int tick = 0;

    while (!g_stop) {
        if (udpSock >= 0) {
            char buffer[256];
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            while (true) {
                int n = recvfrom(udpSock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &client_len);
                if (n > 0) {
                    buffer[n] = '\0';
                    std::string msg(buffer);
                    size_t c1 = msg.find(':');
                    size_t c2 = (c1 != std::string::npos) ? msg.find(':', c1 + 1) : std::string::npos;
                    if (c1 != std::string::npos && c2 != std::string::npos) {
                        std::string cmd = msg.substr(0, c1);
                        int alias = std::stoi(msg.substr(c1 + 1, c2 - c1 - 1));
                        float val = std::stof(msg.substr(c2 + 1));
                        if (alias >= 1 && alias <= motorCount) {
                            int idx = alias - 1;
                            if (std::find(motorIdx.begin(), motorIdx.end(), idx) == motorIdx.end()) {
                                motorIdx.push_back(idx);
                            }
                            if (cmd == "pos") gotoTargets[alias] = val;
                            else if (cmd == "kp") motorKp[idx] = val;
                            else if (cmd == "kd") motorKd[idx] = val;
                            else if (cmd == "tor") motorTor[idx] = val;
                        }
                    }
                } else {
                    break;
                }
            }
        }

        sdk.getMotorActual(actuals);

        for (int i = 0; i < motorCount; i++) {
            targets[i].enabled = (std::find(motorIdx.begin(), motorIdx.end(), i) != motorIdx.end()) ? 1 : 0;
            targets[i].vel = 0;
            targets[i].tor = motorTor[i];
            targets[i].kp = motorKp[i];
            targets[i].kd = motorKd[i];
            
            if (targets[i].enabled) {
                float target_p = currentPos[i];
                if (gotoTargets.count(i + 1) > 0) {
                    target_p = gotoTargets[i + 1];
                }
                
                // Exponential Moving Average (EMA) low-pass filter for silky 50Hz -> 1000Hz interpolation
                // tau ≈ 30ms -> alpha = 1 / (30 + 1) = 0.032f
                const float alpha = 0.032f;
                float ema_target = currentPos[i] * (1.0f - alpha) + target_p * alpha;
                
                // Still apply hard maxStep velocity limit for safety against radical jumps
                currentPos[i] = stepToward(currentPos[i], ema_target, maxStep);
                targets[i].pos = currentPos[i];
            } else {
                targets[i].pos = actuals[i].pos; 
            }
        }
        
        sdk.setMotorTarget(targets);

        tick++;
        if (tick % 20 == 0 && udpSock >= 0) {
            struct sockaddr_in py_addr;
            memset(&py_addr, 0, sizeof(py_addr));
            py_addr.sin_family = AF_INET;
            py_addr.sin_port = htons(8889);
            inet_pton(AF_INET, "127.0.0.1", &py_addr.sin_addr);
            
            std::string sbuf = "batch";
            for (int idx : motorIdx) {
                char mbuf[128];
                snprintf(mbuf, sizeof(mbuf), "|%d:%.4f:%.4f:%.4f:%d:%d",
                    idx + 1, actuals[idx].pos, actuals[idx].vel, actuals[idx].tor,
                    actuals[idx].temp, actuals[idx].errorCode);
                sbuf += mbuf;
            }
            sendto(udpSock, sbuf.c_str(), sbuf.length(), 0, (struct sockaddr *)&py_addr, sizeof(py_addr));
        }

        if (tick % 200 == 0) {
            std::cout << "--- Tick " << tick << " ---\n";
            for (int idx : motorIdx) {
                std::cout << " Alias=" << (idx+1) 
                          << " Cmd=" << targets[idx].pos
                          << " Act=" << actuals[idx].pos 
                          << " Err=" << getErrorString(actuals[idx].errorCode) 
                          << " SW=0x" << std::hex << (actuals[idx].statusWord & 0xFFFF) << std::dec << "\n";
            }
        }
        usleep(periodUs);
    }

    std::cout << "Disabling motors...\n";
    if (udpSock >= 0) close(udpSock);
    
    for(int j=0; j<200; j++) {
        sdk.getMotorActual(actuals);
        for(int i = 0; i < motorCount; i++) targets[i].enabled = 0;
        sdk.setMotorTarget(targets);
        usleep(periodUs);
    }

    return 0;
}
