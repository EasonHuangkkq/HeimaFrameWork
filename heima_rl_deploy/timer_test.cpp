#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

// 简单 1kHz 定时框架测试：测 jitter / 频率稳定性
// 用法：
//   g++ -O2 -std=c++20 timer_test.cpp -o timer_test
//   ./timer_test [period_us=1000] [report_every=1000] [duration_s=10]

static volatile sig_atomic_t g_stop = 0;
static void handleSignal(int) { g_stop = 1; }

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    int period_us = 1000;        // 目标周期：1000us=1kHz
    int report_every = 1000;     // 每 N 次打印一次统计
    int duration_s = 10;         // 运行时长(秒)

    if (argc > 1) period_us = std::max(100, std::atoi(argv[1]));
    if (argc > 2) report_every = std::max(1, std::atoi(argv[2]));
    if (argc > 3) duration_s = std::max(1, std::atoi(argv[3]));

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(period_us);

    auto t0 = clock::now();
    auto next_tick = t0 + period;

    int64_t tick = 0;
    int64_t late_cnt = 0;

    int64_t jitter_max_us = 0;
    int64_t jitter_min_us = 0; // 第一次更新后才有效
    bool jitter_inited = false;

    auto last_report = t0;

    while (!g_stop) {
        auto now = clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - t0).count() >= duration_s) {
            break;
        }

        // “工作负载”放在这里（测试时保持为空）
        std::cout << "test" << std::endl;
        std::cout << "test" << std::endl;

        // ...

        now = clock::now();
        // 相对理想 tick 的误差：now - next_tick（单位 us）
        int64_t err_us = std::chrono::duration_cast<std::chrono::microseconds>(now - next_tick).count();

        if (err_us > 0) late_cnt++;

        if (!jitter_inited) {
            jitter_min_us = err_us;
            jitter_max_us = err_us;
            jitter_inited = true;
        } else {
            if (err_us < jitter_min_us) jitter_min_us = err_us;
            if (err_us > jitter_max_us) jitter_max_us = err_us;
        }

        tick++;

        if (tick % report_every == 0) {
            auto t = clock::now();
            double elapsed_s = std::chrono::duration<double>(t - t0).count();
            double freq = tick / elapsed_s;

            std::cout
                << "tick=" << tick
                << " elapsed=" << elapsed_s << "s"
                << " avg_freq=" << freq << " Hz"
                << " late=" << late_cnt
                << " jitter_us[min,max]=[" << jitter_min_us << "," << jitter_max_us << "]"
                << "\n";

            // 可选：每次报告后重置区间统计
            jitter_inited = false;
            late_cnt = 0;
            last_report = t;
        }

        // 关键：用“绝对时间”睡眠，避免累计漂移
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "Done.\n";
    return 0;
}