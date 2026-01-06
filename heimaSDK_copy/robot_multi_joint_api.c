// robot_multi_joint_api.c — 多关节 EtherCAT 接口（CST 力矩模式）
// 方案 A 最终版（运行中禁止 remap）
//
// 变更要点：
// [MOD] 运行中（robot_is_running()==1）拒绝 robot_map_logical_to_position()，杜绝运行态改映射导致的 TOCTOU 竞态。
// [MOD] robot_set_all_max_torque_percent() 改为复用单轴接口，严格按“逻辑轴”语义更新 g_params（避免写错驱动）。
// [KEEP] 实时循环“快照化”：一次短锁读 motor_states/g_params 与 PDO 反馈，解锁后计算并写 PDO。
// [KEEP] 所有 public API 访问 g_params/motor_states 均在 state_mutex 下。
// [KEEP] CiA-402 推进 0x0006→0x0007→0x000F；运行态维持 0x001F + 周期写 0x6071/0x6072。
// [DOC] 规则：robot_start() 到 robot_stop() 期间禁止 remap；批量接口以逻辑轴为输入，内部调用单轴接口完成映射。

#define _GNU_SOURCE 1

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <math.h>
#include <pthread.h>

#include "ecrt.h"
#include "robot_ethercat_api.h"

//======================= 参数与常量 =======================
#define MAX_SLAVES_CAP 32
#define FREQUENCY      1000
#define CLOCK_TO_USE   CLOCK_MONOTONIC
#define CYCLIC_TORQUE_MODE 10
#define NSEC_PER_SEC   (1000000000L)
#define PERIOD_NS      (NSEC_PER_SEC / FREQUENCY)

// 动态计算，基于额定电流
// MAX_TORQUE 和 MIN_TORQUE 将在运行时根据 rated_I_rms 计算

#define INVALID_INDEX 0xFFFFFFFFu

#define VENDOR_ID  0x00202008
#define PRODUCT_ID 0x00000000

#define SYNC0_SHIFT_NS 250000UL // 250us

//======================= EtherCAT 句柄 =======================
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};
static uint8_t *domain_pd = NULL;
static ec_slave_config_t *sc[MAX_SLAVES_CAP] = {0};
static ec_slave_config_state_t sc_state[MAX_SLAVES_CAP] = {0};
static unsigned g_num_slaves = 0;

//======================= 共享状态与锁 =======================
static motor_state_t motor_states[MAX_SLAVES_CAP] = {0};
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER; // 保护 motor_states + g_params

// 轴映射表（逻辑轴 <-> 物理位置）
// 方案 A：运行中不改映射，因此无需复杂的多把锁；读写在停机态完成即可。
static unsigned g_logical_to_pos[MAX_SLAVES_CAP];
static unsigned g_pos_to_logical[MAX_SLAVES_CAP];

//======================= 轴参数（额定电流/KT/Max%） =======================
typedef struct {
    double rated_I_rms;   // A
    double kt_out;        // Nm/A
    double max_percent;   // %
    char   model[48];
} motor_param_t;

static motor_param_t g_params[MAX_SLAVES_CAP];

// 关节限位/零位参数（受 state_mutex 保护）
typedef struct {
    int      has_limits;       // 是否启用限位
    int      has_home;         // 是否已捕获/设置零位
    int32_t  home_offset_inc;  // 作为零位的编码器计数（绝对计数）
    int32_t  min_rel_inc;      // 相对零位的最小计数
    int32_t  max_rel_inc;      // 相对零位的最大计数
    int      one_sided;        // 单边限幅：只阻止“继续外推”的力矩
    int      soft_zone_enabled;// 是否启用软区
    int32_t  soft_zone_inc;    // 软区宽度（计数），靠近边界按距离线性衰减外推力矩
} joint_limit_t;

static joint_limit_t g_limits[MAX_SLAVES_CAP] = {0};

// 默认电机参数
static void init_default_params(unsigned n) {
    for (unsigned i=0; i<n; ++i) {
        g_params[i].rated_I_rms = 10.0;
        g_params[i].kt_out      = 1.0;
        g_params[i].max_percent = 100.0;
        snprintf(g_params[i].model, sizeof(g_params[i].model), "axis-%u", i);
    }
}

// 初始化 1:1 映射（停机态调用）
static void init_default_mapping(unsigned slaves) {
    for (unsigned i = 0; i < MAX_SLAVES_CAP; ++i) {
        g_logical_to_pos[i] = INVALID_INDEX;
        g_pos_to_logical[i] = INVALID_INDEX;
    }
    for (unsigned i = 0; i < slaves && i < MAX_SLAVES_CAP; ++i) {
        g_logical_to_pos[i] = i;
        g_pos_to_logical[i] = i;
    }
}

// 逻辑轴 -> 物理位置（运行中只读，不改映射）
static int logical_to_position(unsigned logic, unsigned *pos_out) {
    if (!pos_out) return -1;
    if (logic >= g_num_slaves) {
        fprintf(stderr, "[MAP] logic %u out of range (0..%u)\n", logic, g_num_slaves ? g_num_slaves - 1 : 0);
        return -1;
    }
    unsigned pos = g_logical_to_pos[logic];
    if (pos >= g_num_slaves) {
        fprintf(stderr, "[MAP] logic %u not mapped (pos=%u invalid)\n", logic, pos);
        return -1;
    }
    *pos_out = pos;
    return 0;
}

//======================= PDO 偏移 =======================
typedef struct {
    unsigned int ctrl_word;                  // 0x6040 U16
    unsigned int target_position;            // 0x607A S32
    unsigned int target_velocity;            // 0x60FF S32
    unsigned int target_torque;              // 0x6071 S16
    unsigned int max_torque;                 // 0x6072 U16
    unsigned int operation_mode;             // 0x6060 S8
    unsigned int status_word;                // 0x6041 U16
    unsigned int position_actual_value;      // 0x6064 S32
    unsigned int velocity_actual_value;      // 0x606C S32
    unsigned int torque_actual_value;        // 0x6077 S16
    unsigned int error_code;                 // 0x603F U16
    unsigned int modes_of_operation_display; // 0x6061 S8
} offsets_t;

static offsets_t ofs[MAX_SLAVES_CAP];

typedef struct { ec_pdo_entry_reg_t *list; size_t count; } domain_regs_buf_t;

//======================= 工具函数 =======================
static inline uint64_t TIMESPEC2NS(struct timespec t) {
    return (uint64_t)t.tv_sec * NSEC_PER_SEC + (uint64_t)t.tv_nsec;
}

static inline uint8_t cia402_operation_enabled(uint16_t status_word) {
    uint16_t bits = status_word & 0x006F;
    if ((bits & 0x0007) != 0x0007) return 0;
    if (bits & 0x0048) return 0;
    return 1;
}

static void check_domain_state(void) {
    ec_domain_state_t ds;
    ecrt_domain_state(domain, &ds);
    if (ds.working_counter != domain_state.working_counter) printf("Domain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state.wc_state) printf("Domain: State %u.\n", ds.wc_state);
    domain_state = ds;
}
static void check_master(void) {
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding) printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states) printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up) printf("Link is %s.\n", ms.link_up ? "up" : "down");
    master_state = ms;
}
static void check_sc(unsigned s) {
    ec_slave_config_state_t st;
    ecrt_slave_config_state(sc[s], &st);
    if (st.al_state != sc_state[s].al_state) printf("[S%u] State 0x%02X.\n", s, st.al_state);
    if (st.online != sc_state[s].online) printf("[S%u] %s.\n", s, st.online ? "online" : "offline");
    if (st.operational != sc_state[s].operational) printf("[S%u] %soperational.\n", s, st.operational ? "" : "Not ");
    sc_state[s] = st;
}

// 简单探测从站数（用于 g_num_slaves）
static unsigned detect_slaves_by_tool(void) {
    FILE *fp = popen("/usr/local/bin/ethercat slaves", "r");
    if (!fp) return 0;
    unsigned n = 0; char line[512];
    while (fgets(line, sizeof(line), fp)) if (isdigit((unsigned char)line[0])) n++;
    pclose(fp);
    return n;
}

// 注册 PDO 条目：激活后 ofs[*] 会被主站写入域内偏移
static int make_domain_regs(unsigned slaves, domain_regs_buf_t *buf) {
    const int ENTRIES_PER_SLAVE = 12; // 6 Rx + 6 Tx
    size_t total = slaves * ENTRIES_PER_SLAVE + 1;
    buf->list = (ec_pdo_entry_reg_t*)calloc(total, sizeof(ec_pdo_entry_reg_t));
    if (!buf->list) return -1;

    size_t k = 0;
    for (unsigned s = 0; s < slaves; ++s) {
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6040,.subindex=0,.offset=&ofs[s].ctrl_word,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x607A,.subindex=0,.offset=&ofs[s].target_position,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x60FF,.subindex=0,.offset=&ofs[s].target_velocity,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6071,.subindex=0,.offset=&ofs[s].target_torque,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6072,.subindex=0,.offset=&ofs[s].max_torque,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6060,.subindex=0,.offset=&ofs[s].operation_mode,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6041,.subindex=0,.offset=&ofs[s].status_word,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6064,.subindex=0,.offset=&ofs[s].position_actual_value,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x606C,.subindex=0,.offset=&ofs[s].velocity_actual_value,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6077,.subindex=0,.offset=&ofs[s].torque_actual_value,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x603F,.subindex=0,.offset=&ofs[s].error_code,.bit_position=NULL };
        buf->list[k++] = (ec_pdo_entry_reg_t){ .alias=0,.position=(int)s,.vendor_id=VENDOR_ID,.product_code=PRODUCT_ID, .index=0x6061,.subindex=0,.offset=&ofs[s].modes_of_operation_display,.bit_position=NULL };
    }

    buf->list[k] = (ec_pdo_entry_reg_t){0};
    buf->count = k + 1;
    return 0;
}

//======================= 换算（Nm ↔ raw‰） =======================

// 根据额定电流计算最大/最小力矩限制
static inline int16_t get_max_torque_raw(double rated_I_rms) {
    return (int16_t)lrint(rated_I_rms * 100.0); // 额定电流的 0.01A 单位
}
static inline int16_t get_min_torque_raw(double rated_I_rms) {
    return -(int16_t)lrint(rated_I_rms * 100.0); // 负的额定电流
}

static inline int16_t nm_to_raw(double torque_nm, const motor_param_t *pm) {
    double current_amps = torque_nm / pm->kt_out;  // 需要的电流 (A)
    double raw = current_amps * 100.0;  // 转换为 0.01A 单位 (500 = 5A)
    int16_t max_raw = get_max_torque_raw(pm->rated_I_rms);
    int16_t min_raw = get_min_torque_raw(pm->rated_I_rms);
    if (raw > max_raw) raw = max_raw;
    if (raw < min_raw) raw = min_raw;
    return (int16_t)lrint(raw);
}
static inline double raw_to_nm(int16_t raw, const motor_param_t *pm) {
    double current_amps = (double)raw / 100.0;  // 0.01A 单位转电流 (A)
    return current_amps * pm->kt_out;  // 电流转力矩 (Nm)
}
static inline uint16_t current_to_0x6072(double current_amps) {
    if (current_amps < 0.0) current_amps = 0.0;
    return (uint16_t)lrint(current_amps * 100.0); // 转换为 0.01A 单位
}

//======================= 停机流程 =======================
static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig) { (void)sig; g_stop = 1; }
static const struct timespec cycletime = {0, PERIOD_NS};

static void safe_stop_all(unsigned slaves, unsigned grace_cycles) {
    printf("\n[STOP] 优雅停机：力矩清零...\n");
    pthread_mutex_lock(&state_mutex);
    for (unsigned s=0;s<slaves;++s) {
        motor_states[s].target_torque = 0;
        motor_states[s].target_torque_nm = 0.0;
    }
    pthread_mutex_unlock(&state_mutex);

    for (unsigned i=0;i<grace_cycles;++i) {
        for (unsigned s=0;s<slaves;++s) {
            EC_WRITE_S16(domain_pd + ofs[s].target_torque, 0);
            EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x001F);
        }
        ecrt_domain_queue(domain); ecrt_master_send(master);
        struct timespec t; clock_gettime(CLOCK_TO_USE,&t);
        ecrt_master_application_time(master, TIMESPEC2NS(t));
        ecrt_master_receive(master); ecrt_domain_process(domain);
        clock_nanosleep(CLOCK_TO_USE, 0, &cycletime, NULL);
    }

    printf("[STOP] 退出运行态 -> Switched on\n");
    for (unsigned s=0;s<slaves;++s) EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x0007);
    ecrt_domain_queue(domain); ecrt_master_send(master); usleep(5000);

    printf("[STOP] 关使能 -> Ready to switch on\n");
    for (unsigned s=0;s<slaves;++s) EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x0006);
    ecrt_domain_queue(domain); ecrt_master_send(master); usleep(5000);

    printf("[STOP] 完成。\n");
}

//======================= 实时主循环（快照化 + 402 推进） =======================
static void cyclic_task(unsigned slaves) {
    static uint16_t cmd_mask[MAX_SLAVES_CAP] = {0};
    static uint8_t  op_ready[MAX_SLAVES_CAP] = {0};
    static uint8_t  started = 0;
    static int      init = 0;

    const unsigned REFCLK_SYNC_DIV = FREQUENCY;
    static unsigned refclk_sync_counter = 0;

    uint64_t total_samples = 0;
    double total_sum_perr = 0.0, total_sumsq_perr = 0.0;
    uint32_t total_perr_max = 0;

    uint64_t samples = 0;
    double sum_perr = 0.0, sumsq_perr = 0.0;
    uint32_t perr_min = 0xffffffff, perr_max = 0;
    struct timespec lastStart = {0}, start={0};

    if (!init) { for (unsigned s=0;s<slaves;++s) cmd_mask[s]=0x004F; init = 1; }

    struct timespec wake;
    clock_gettime(CLOCK_TO_USE, &wake);
    unsigned once_per_sec = 0;

    printf("\n========== EtherCAT 实时循环启动（CST） ==========\n");
    printf("控制频率: %d Hz (周期 %u ns)\n", FREQUENCY, (uint32_t)PERIOD_NS);
    printf("轴数: %u\n", slaves);
    printf("模式: CST (0x6060=10), 周期写 0x6071 目标力矩\n");
    printf("===============================================\n\n");

    typedef struct {
        int16_t  tgt_raw;
        double   I_rms;
        double   Kt;
        uint16_t max_6072;
        uint16_t status_word;
        int32_t  pos_actual;
        int32_t  vel_actual;
        int16_t  trq_actual;
        uint16_t err_code;
    } snap_t;
    snap_t snap[MAX_SLAVES_CAP];

    while (!g_stop) {
        // 定时
        wake.tv_nsec += PERIOD_NS;
        while (wake.tv_nsec >= NSEC_PER_SEC) { wake.tv_nsec -= NSEC_PER_SEC; wake.tv_sec++; }
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake, NULL);

        ecrt_master_application_time(master, TIMESPEC2NS(wake));

        // 抖动统计
        clock_gettime(CLOCK_TO_USE, &start);
        if (lastStart.tv_sec!=0 || lastStart.tv_nsec!=0) {
            uint64_t p_ns = TIMESPEC2NS(start) - TIMESPEC2NS(lastStart);
            int64_t  perr = (int64_t)p_ns - (int64_t)PERIOD_NS;
            uint32_t aerr = (perr>=0)? (uint32_t)perr : (uint32_t)(-perr);
            if (aerr > perr_max) perr_max = aerr;
            if (aerr < perr_min) perr_min = aerr;
            sum_perr   += (double)perr;
            sumsq_perr += (double)perr*(double)perr;
            samples++;
            if (aerr > total_perr_max) total_perr_max = aerr;
            total_sum_perr   += (double)perr;
            total_sumsq_perr += (double)perr*(double)perr;
            total_samples++;
        }

        // 收/处理上一拍
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        // 一次短锁：制作“整拍快照”
        pthread_mutex_lock(&state_mutex);
        for (unsigned s=0; s<slaves; ++s) {
            snap[s].status_word = EC_READ_U16(domain_pd + ofs[s].status_word);
            snap[s].pos_actual  = EC_READ_S32(domain_pd + ofs[s].position_actual_value);
            snap[s].vel_actual  = EC_READ_S32(domain_pd + ofs[s].velocity_actual_value);
            snap[s].trq_actual  = EC_READ_S16(domain_pd + ofs[s].torque_actual_value);
            snap[s].err_code    = EC_READ_U16(domain_pd + ofs[s].error_code);

            int16_t t = motor_states[s].target_torque;
            int16_t max_raw = get_max_torque_raw(g_params[s].rated_I_rms);
            int16_t min_raw = get_min_torque_raw(g_params[s].rated_I_rms);
            if (t < min_raw) t = min_raw;
            if (t > max_raw) t = max_raw;

            // 关节限位（相对零位）。支持：单边限幅 + 软区渐进衰减
            if (g_limits[s].has_limits && g_limits[s].has_home) {
                int32_t rel_inc = snap[s].pos_actual - g_limits[s].home_offset_inc;
                int one_sided = g_limits[s].one_sided;
                int soft_en   = g_limits[s].soft_zone_enabled;
                int32_t soft  = g_limits[s].soft_zone_inc;

                // 越界：根据单边/双边策略处理
                if (rel_inc > g_limits[s].max_rel_inc) {
                    if (one_sided) {
                        if (t > 0) t = 0; // 仅阻止继续向外的正向力矩
                    } else {
                        t = 0; // 双边：越界直接清零
                    }
                } else if (rel_inc < g_limits[s].min_rel_inc) {
                    if (one_sided) {
                        if (t < 0) t = 0; // 仅阻止继续向外的负向力矩
                    } else {
                        t = 0;
                    }
                } else if (soft_en && soft > 0) {
                    // 在软区内：靠近上限时削减正向力矩；靠近下限时削减负向力矩
                    int32_t dist_upper = g_limits[s].max_rel_inc - rel_inc;
                    if (dist_upper >= 0 && dist_upper <= soft && t > 0) {
                        double scale = (double)dist_upper / (double)soft; // [0,1]
                        t = (int16_t)lrint((double)t * scale);
                    }
                    int32_t dist_lower = rel_inc - g_limits[s].min_rel_inc;
                    if (dist_lower >= 0 && dist_lower <= soft && t < 0) {
                        double scale = (double)dist_lower / (double)soft; // [0,1]
                        t = (int16_t)lrint((double)t * scale);
                    }
                }
            }

            snap[s].tgt_raw  = t;
            snap[s].I_rms    = g_params[s].rated_I_rms;
            snap[s].Kt       = g_params[s].kt_out;
            snap[s].max_6072 = current_to_0x6072(g_params[s].rated_I_rms * g_params[s].max_percent / 100.0);
        }
        pthread_mutex_unlock(&state_mutex);

        // 每秒打印一次
        if (++once_per_sec >= FREQUENCY) {
            once_per_sec = 0;
            check_domain_state();
            check_master();
            for (unsigned s=0;s<slaves;++s) check_sc(s);
            if (samples > 0) {
                double mean = sum_perr / (double)samples;
                double rms  = sqrt(sumsq_perr / (double)samples);
                printf("[JITTER] max=%u ns | mean=%.1f ns | rms=%.1f ns\n", perr_max, mean, rms);

                double t_nm0 = (snap[0].tgt_raw / 100.0) * snap[0].Kt;
                double a_nm0 = (snap[0].trq_actual / 100.0) * snap[0].Kt;
                printf("[状态] 轴0: τ*=%d (%.2f Nm), τ=%d (%.2f Nm), vel=%d, pos=%d, SW=0x%04X, OP=%d\n",
                       snap[0].tgt_raw, t_nm0, snap[0].trq_actual, a_nm0,
                       snap[0].vel_actual, snap[0].pos_actual,
                       snap[0].status_word, cia402_operation_enabled(snap[0].status_word));
            }
            samples=0; sum_perr=0; sumsq_perr=0; perr_min=0xffffffff; perr_max=0;
        }

        // 402 推进（基于快照的 SW）
        for (unsigned s=0; s<slaves; ++s) {
            uint16_t st = snap[s].status_word;

            // Fault -> Fault reset
            if (st & 0x0008) { EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x0080); op_ready[s] = 0; continue; }

            if ((st & cmd_mask[s]) == 0x0001) {
                EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x0006); // Shutdown
                EC_WRITE_S8 (domain_pd + ofs[s].operation_mode, CYCLIC_TORQUE_MODE);
                EC_WRITE_S16(domain_pd + ofs[s].target_torque, 0);
                EC_WRITE_U16(domain_pd + ofs[s].max_torque,   snap[s].max_6072);
                cmd_mask[s] = 0x006F;
                op_ready[s] = 0;
                continue;
            }
            if ((st & cmd_mask[s]) == 0x0021) {
                EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x0007); // Switch on
                op_ready[s] = 0;
                continue;
            }
            if ((st & cmd_mask[s]) == 0x0023) {
                EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x000F); // Enable operation
                op_ready[s] = 0;
                continue;
            }
            if ((st & cmd_mask[s]) == 0x0027) {
                EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x001F); // Enable + Set-point enable
                op_ready[s] = 1;
            } else {
                op_ready[s] = 0;
            }
        }

        // 同步屏障
        if (!started) {
            int all_ready = 1; for (unsigned s=0;s<slaves;++s) if (!op_ready[s]) { all_ready = 0; break; }
            if (all_ready) { printf("\n[SYNC] 所有轴已到 OP Enabled（CST），开始接受外部力矩指令\n\n"); started = 1; }
        } else {
            // 写目标（基于快照）
            for (unsigned s=0; s<slaves; ++s) {
                EC_WRITE_U16(domain_pd + ofs[s].ctrl_word, 0x001F);
                EC_WRITE_S16(domain_pd + ofs[s].target_torque, snap[s].tgt_raw);
                EC_WRITE_U16(domain_pd + ofs[s].max_torque,    snap[s].max_6072);
            }

            // 回填显示用状态（短锁）
            pthread_mutex_lock(&state_mutex);
            for (unsigned s=0; s<slaves; ++s) {
                motor_states[s].actual_velocity  = snap[s].vel_actual;
                motor_states[s].actual_position  = snap[s].pos_actual;
                motor_states[s].actual_torque    = snap[s].trq_actual;
                motor_states[s].status_word      = snap[s].status_word;
                motor_states[s].error_code       = snap[s].err_code;
                motor_states[s].target_torque_nm = (snap[s].tgt_raw / 100.0) * snap[s].Kt;
                motor_states[s].actual_torque_nm = (snap[s].trq_actual / 100.0) * snap[s].Kt;
                motor_states[s].operational      = cia402_operation_enabled(snap[s].status_word);
            }
            pthread_mutex_unlock(&state_mutex);

            int all_still_ready = 1; for (unsigned s=0;s<slaves;++s) if (!op_ready[s]) { all_still_ready = 0; break; }
            if (!all_still_ready) { started = 0; printf("[SYNC] 检测到从站掉出 OP，重启同步屏障...\n"); }
        }

        // DC 同步 & 发送
        if (++refclk_sync_counter >= REFCLK_SYNC_DIV) {
            struct timespec ref_time; refclk_sync_counter = 0; clock_gettime(CLOCK_TO_USE, &ref_time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(ref_time));
        }
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        lastStart = start;
    }

    double total_mean = total_samples ? total_sum_perr / (double)total_samples : 0.0;
    double total_rms  = total_samples ? sqrt(total_sumsq_perr / (double)total_samples) : 0.0;
    safe_stop_all(slaves, (unsigned)(0.5 * FREQUENCY));
    printf("\n[统计] Jitter: max=%u ns | mean=%.1f ns | rms=%.1f ns (N=%lu)\n",
           total_perr_max, total_mean, total_rms, (unsigned long)total_samples);
}

//======================= 线程/域/主站初始化 =======================
static pthread_t g_rt_thread;
static int g_thread_started = 0;
static domain_regs_buf_t g_regs = {0};

int robot_init(void) {
    struct sigaction sa = {.sa_handler = on_signal};
    sigemptyset(&sa.sa_mask); sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { perror("mlockall"); return -1; }

    unsigned slaves = detect_slaves_by_tool();
    if (slaves == 0) { fprintf(stderr, "未检测到从站。\n"); return -1; }
    if (slaves > MAX_SLAVES_CAP) slaves = MAX_SLAVES_CAP;
    g_num_slaves = slaves;
    printf("检测到从站数量: %u\n", slaves);

    init_default_params(g_num_slaves);
    init_default_mapping(g_num_slaves);

    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "ecrt_request_master 失败\n"); return -1; }

    domain = ecrt_master_create_domain(master);
    if (!domain) { fprintf(stderr, "ecrt_master_create_domain 失败\n"); return -1; }

    for (unsigned s=0;s<slaves;++s) {
        sc[s] = ecrt_master_slave_config(master, 0, s, VENDOR_ID, PRODUCT_ID);
        if (!sc[s]) { fprintf(stderr, "slave_config 失败：pos=%u\n", s); return -1; }

        if (ecrt_slave_config_dc(sc[s], 0x0300, PERIOD_NS, SYNC0_SHIFT_NS, 0, 0)) {
            fprintf(stderr, "DC 配置失败：pos=%u\n", s); return -1;
        }
    }

    if (ecrt_master_select_reference_clock(master, sc[0])) {
        fprintf(stderr, "选择参考钟失败\n"); return -1;
    }

    if (make_domain_regs(slaves, &g_regs)) { fprintf(stderr, "寄存表构建失败\n"); return -1; }
    if (ecrt_domain_reg_pdo_entry_list(domain, g_regs.list)) {
        fprintf(stderr, "PDO entry 注册失败\n"); free(g_regs.list); g_regs.list = NULL; return -1;
    }

    if (ecrt_master_activate(master)) {
        fprintf(stderr, "ecrt_master_activate 失败\n"); free(g_regs.list); g_regs.list = NULL; return -1;
    }

    domain_pd = ecrt_domain_data(domain);
    if (!domain_pd) { fprintf(stderr, "ecrt_domain_data 失败\n"); free(g_regs.list); g_regs.list = NULL; return -1; }

    struct sched_param sp = {.sched_priority = sched_get_priority_max(SCHED_FIFO)};
    printf("Using priority %d\n", sp.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) perror("sched_setscheduler");

    return 0;
}

static void* rt_thread_entry(void* arg) { (void)arg; cyclic_task(g_num_slaves); return NULL; }

int robot_start(void) {
    if (g_thread_started) return 0;
    g_stop = 0;

    cpu_set_t cs;
    CPU_ZERO(&cs);
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    if (ncpu < 1) ncpu = 1;
    int start_cpu = 8;
    int end_cpu   = 13;
    if (start_cpu >= ncpu) {
        fprintf(stderr, "[WARN] 可用 CPU 少于 9 个，无法按 8~13 绑定，当前 ncpu=%ld，将不设亲和（由内核调度）。\n", ncpu);
    } else {
        if (end_cpu >= ncpu) end_cpu = (int)ncpu - 1;
        for (int cpu = start_cpu; cpu <= end_cpu; ++cpu) {
            CPU_SET(cpu, &cs);
        }
    }

    int rc = pthread_create(&g_rt_thread, NULL, rt_thread_entry, NULL);
    if (rc != 0) { perror("pthread_create"); return -1; }

    if (CPU_COUNT(&cs) > 0) {
        (void)pthread_setaffinity_np(g_rt_thread, sizeof(cs), &cs);
    }

    g_thread_started = 1;
    return 0;
}

void robot_stop(void) {
    if (!g_thread_started) return;
    g_stop = 1;
    pthread_join(g_rt_thread, NULL);
    g_thread_started = 0;
}

void robot_release(void) {
    if (g_regs.list) { free(g_regs.list); g_regs.list = NULL; }
    if (master) { ecrt_release_master(master); master = NULL; }
    printf("Master released. Bye.\n");
}

//======================= API 实现 =======================
int robot_set_motor_torque(unsigned motor_id, int16_t torque_raw) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) {
        fprintf(stderr, "[API] 错误：逻辑轴ID %u 超出范围 (0~%u)\n",
                motor_id, g_num_slaves ? g_num_slaves - 1 : 0);
        return -1;
    }
    int16_t max_raw = get_max_torque_raw(g_params[pos].rated_I_rms);
    int16_t min_raw = get_min_torque_raw(g_params[pos].rated_I_rms);
    if (torque_raw < min_raw) torque_raw = min_raw;
    if (torque_raw > max_raw) torque_raw = max_raw;

    pthread_mutex_lock(&state_mutex);
    motor_states[pos].target_torque = torque_raw;
    motor_states[pos].target_torque_nm = raw_to_nm(torque_raw, &g_params[pos]);
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_set_all_torques(const int16_t *torques_raw) {
    if (!torques_raw) return -1;
    pthread_mutex_lock(&state_mutex);
    for (unsigned logic = 0; logic < g_num_slaves; ++logic) {
        unsigned pos;
        if (logical_to_position(logic, &pos) != 0) continue;
        int16_t t = torques_raw[logic];
        int16_t max_raw = get_max_torque_raw(g_params[pos].rated_I_rms);
        int16_t min_raw = get_min_torque_raw(g_params[pos].rated_I_rms);
        if (t < min_raw) t = min_raw;
        if (t > max_raw) t = max_raw;
        motor_states[pos].target_torque = t;
        motor_states[pos].target_torque_nm = raw_to_nm(t, &g_params[pos]);
    }
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_set_motor_torque_nm(unsigned motor_id, double torque_nm) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    int16_t raw = nm_to_raw(torque_nm, &g_params[pos]);
    motor_states[pos].target_torque = raw;
    motor_states[pos].target_torque_nm = torque_nm;
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_set_all_torques_nm(const double *torques_nm) {
    if (!torques_nm) return -1;
    pthread_mutex_lock(&state_mutex);
    for (unsigned logic = 0; logic < g_num_slaves; ++logic) {
        unsigned pos;
        if (logical_to_position(logic, &pos) != 0) continue;
        int16_t raw = nm_to_raw(torques_nm[logic], &g_params[pos]);
        motor_states[pos].target_torque = raw;
        motor_states[pos].target_torque_nm = torques_nm[logic];
    }
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_set_max_torque_percent(unsigned motor_id, double percent) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    // 将百分比转换为最大电流限制
    g_params[pos].max_percent = percent;
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

// [MOD] 复用单轴接口，保证按逻辑轴语义更新参数，避免写错驱动。
int robot_set_all_max_torque_percent(const double *percents, unsigned count) {
    if (!percents) return -1;
    if (count > g_num_slaves) count = g_num_slaves;
    for (unsigned i=0; i<count; ++i) {
        (void)robot_set_max_torque_percent(i, percents[i]); // 复用单轴接口（内部做映射）
    }
    return 0;
}

int robot_configure_motor(unsigned motor_id, double rated_current_rms,
                          double torque_constant_nm_per_amp, const char *model_name) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    if (rated_current_rms <= 0.0 || torque_constant_nm_per_amp <= 0.0) return -2;
    pthread_mutex_lock(&state_mutex);
    g_params[pos].rated_I_rms = rated_current_rms;
    g_params[pos].kt_out      = torque_constant_nm_per_amp;
    if (model_name && *model_name) snprintf(g_params[pos].model, sizeof(g_params[pos].model), "%s", model_name);
    else snprintf(g_params[pos].model, sizeof(g_params[pos].model), "axis-%u", motor_id);
    // 以新参数刷新显示用 Nm
    motor_states[pos].target_torque_nm = raw_to_nm(motor_states[pos].target_torque, &g_params[pos]);
    motor_states[pos].actual_torque_nm = raw_to_nm(motor_states[pos].actual_torque, &g_params[pos]);
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

// [MOD] 运行中禁止改映射：硬防呆。需要改映射：请在 robot_start() 前或 robot_stop() 后调用。
int robot_map_logical_to_position(unsigned logical_id, unsigned position) {
    if (robot_is_running()) { // 运行态直接拒绝
        fprintf(stderr, "[MAP] reject: cannot remap while running\n");
        return -1;
    }
    if (logical_id >= g_num_slaves || position >= g_num_slaves) return -1;

    unsigned prev_logic = g_pos_to_logical[position];
    if (prev_logic < g_num_slaves) g_logical_to_pos[prev_logic] = INVALID_INDEX;

    unsigned prev_pos = g_logical_to_pos[logical_id];
    if (prev_pos < g_num_slaves) g_pos_to_logical[prev_pos] = INVALID_INDEX;

    g_logical_to_pos[logical_id] = position;
    g_pos_to_logical[position]   = logical_id;

    printf("[MAP] logic %u -> pos %u\n", logical_id, position);
    return 0;
}

int robot_get_motor_torque_nm(unsigned motor_id, double *target_nm, double *actual_nm) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    if (target_nm) *target_nm = motor_states[pos].target_torque_nm;
    if (actual_nm) *actual_nm = motor_states[pos].actual_torque_nm;
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_convert_torque_raw_to_nm(unsigned motor_id, int16_t raw, double *out_nm) {
    unsigned pos;
    if (!out_nm || logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    *out_nm = raw_to_nm(raw, &g_params[pos]);
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_convert_torque_nm_to_raw(unsigned motor_id, double torque_nm, int16_t *out_raw) {
    unsigned pos;
    if (!out_raw || logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    *out_raw = nm_to_raw(torque_nm, &g_params[pos]);
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

int robot_get_motor_state(unsigned motor_id, motor_state_t *state) {
    unsigned pos;
    if (!state || logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    *state = motor_states[pos];
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

unsigned robot_get_all_states(motor_state_t *states) {
    if (!states) return 0;
    pthread_mutex_lock(&state_mutex);
    for (unsigned logic = 0; logic < g_num_slaves; ++logic) {
        unsigned pos;
        if (logical_to_position(logic, &pos) == 0) {
            states[logic] = motor_states[pos];
        } else {
            memset(&states[logic], 0, sizeof(motor_state_t));
        }
    }
    pthread_mutex_unlock(&state_mutex);
    return g_num_slaves;
}

unsigned robot_get_num_motors(void) { return g_num_slaves; }

void robot_emergency_stop(void) {
    pthread_mutex_lock(&state_mutex);
    for (unsigned i=0;i<g_num_slaves;i++) {
        motor_states[i].target_torque = 0;
        motor_states[i].target_torque_nm = 0.0;
    }
    pthread_mutex_unlock(&state_mutex);
    printf("[API] 急停触发！（目标力矩清零）\n");
}

int robot_is_running(void) {
    return g_thread_started && !g_stop;
}

//======================= 关节限位/零位 API =======================
int robot_capture_home_position(unsigned motor_id) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].home_offset_inc = motor_states[pos].actual_position;
    g_limits[pos].has_home = 1;
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u capture home: inc=%d\n", motor_id, (int)g_limits[pos].home_offset_inc);
    return 0;
}

int robot_set_home_offset_inc(unsigned motor_id, int32_t home_offset_inc) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].home_offset_inc = home_offset_inc;
    g_limits[pos].has_home = 1;
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u set home: inc=%d\n", motor_id, (int)home_offset_inc);
    return 0;
}

int robot_configure_joint_limits_inc(unsigned motor_id, int32_t min_rel_inc, int32_t max_rel_inc) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    if (min_rel_inc >= max_rel_inc) return -2;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].min_rel_inc = min_rel_inc;
    g_limits[pos].max_rel_inc = max_rel_inc;
    g_limits[pos].has_limits  = 1;
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u limits: [%d, %d] rel_inc\n", motor_id, (int)min_rel_inc, (int)max_rel_inc);
    return 0;
}

int robot_clear_joint_limits(unsigned motor_id) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].has_limits = 0;
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u limits cleared\n", motor_id);
    return 0;
}

int robot_configure_joint_soft_zone_inc(unsigned motor_id, int32_t soft_zone_inc) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    if (soft_zone_inc < 0) return -2;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].soft_zone_inc     = soft_zone_inc;
    g_limits[pos].soft_zone_enabled = (soft_zone_inc > 0);
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u soft-zone: %d inc\n", motor_id, (int)soft_zone_inc);
    return 0;
}

int robot_configure_limit_behavior(unsigned motor_id, int one_sided, int enable_soft_zone) {
    unsigned pos;
    if (logical_to_position(motor_id, &pos) != 0) return -1;
    pthread_mutex_lock(&state_mutex);
    g_limits[pos].one_sided         = one_sided ? 1 : 0;
    g_limits[pos].soft_zone_enabled = enable_soft_zone ? 1 : 0;
    pthread_mutex_unlock(&state_mutex);
    printf("[LIMIT] axis %u behavior: one_sided=%d, soft_zone_enabled=%d\n", motor_id, g_limits[pos].one_sided, g_limits[pos].soft_zone_enabled);
    return 0;
}

#ifdef BUILD_STANDALONE
static pthread_t g_rt_thread;
int main(int argc, char **argv) {
    (void)argc; (void)argv;
    if (robot_init() != 0) return -1;
    if (robot_start() != 0) return -1;
    pthread_join(g_rt_thread, NULL);
    robot_release();
    return 0;
}
#endif
