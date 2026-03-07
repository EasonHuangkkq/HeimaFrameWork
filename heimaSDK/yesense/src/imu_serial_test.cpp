#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

#include "yesense_decoder_comm.h"
#include "yesense_decoder.h"

namespace {

struct BaudMap {
    int baud;
    speed_t speed;
};

static speed_t baud_to_speed(int baud) {
    static const BaudMap kBaudMap[] = {
        {9600, B9600},
        {19200, B19200},
        {38400, B38400},
        {57600, B57600},
        {115200, B115200},
#ifdef B230400
        {230400, B230400},
#endif
#ifdef B460800
        {460800, B460800},
#endif
#ifdef B921600
        {921600, B921600},
#endif
    };

    for (const auto &entry : kBaudMap) {
        if (entry.baud == baud) {
            return entry.speed;
        }
    }
    return B0;
}

static bool apply_custom_baud(int fd, int baud) {
#ifdef __APPLE__
    speed_t speed = static_cast<speed_t>(baud);
    if (ioctl(fd, IOSSIOSPEED, &speed) != 0) {
        std::perror("ioctl IOSSIOSPEED");
        return false;
    }
#else
    (void)fd;
    (void)baud;
#endif
    return true;
}

static bool has_header(const unsigned char *data, size_t len) {
    if (len < 2) {
        return false;
    }
    for (size_t i = 0; i + 1 < len; ++i) {
        if (data[i] == 0x59 && data[i + 1] == 0x53) {
            return true;
        }
    }
    return false;
}

static void dump_hex(const unsigned char *data, size_t len, int max_len) {
    size_t out_len = len;
    if (max_len > 0 && out_len > static_cast<size_t>(max_len)) {
        out_len = static_cast<size_t>(max_len);
    }

    std::printf("raw %zu bytes%s:", len, has_header(data, len) ? " (YS header)" : "");
    for (size_t i = 0; i < out_len; ++i) {
        std::printf(" %02X", data[i]);
    }
    if (out_len < len) {
        std::printf(" ...");
    }
    std::printf("\n");
    std::fflush(stdout);
}

static int open_serial(const char *port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::perror("open");
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        std::perror("tcgetattr");
        close(fd);
        return -1;
    }

    speed_t speed = baud_to_speed(baud);
    if (speed == B0) {
#ifndef __APPLE__
        std::fprintf(stderr, "Unsupported baud rate: %d\n", baud);
        close(fd);
        return -1;
#else
        speed = B115200;
#endif
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1; // 100ms timeout

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::perror("tcsetattr");
        close(fd);
        return -1;
    }

    if (!apply_custom_baud(fd, baud)) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIFLUSH);
    return fd;
}

static void print_sample(const yesense::yis_out_data_t &out) {
    std::printf("tid=%u", out.tid);
    if (out.content.acc) {
        std::printf(" acc=%.6f %.6f %.6f", out.acc.x, out.acc.y, out.acc.z);
    }
    if (out.content.gyro) {
        std::printf(" gyro=%.6f %.6f %.6f", out.gyro.x, out.gyro.y, out.gyro.z);
    }
    if (out.content.euler) {
        std::printf(" euler=%.6f %.6f %.6f", out.euler.pitch, out.euler.roll, out.euler.yaw);
    }
    if (out.content.quat) {
        std::printf(" quat=%.6f %.6f %.6f %.6f",
                    out.quat.q0, out.quat.q1, out.quat.q2, out.quat.q3);
    }
    if (out.content.sensor_temp) {
        std::printf(" temp=%.2f", out.sensor_temp);
    }
    if (out.content.sample_timestamp) {
        std::printf(" ts=%u", out.sample_timestamp);
    }
    std::printf("\n");
    std::fflush(stdout);
}

static void print_usage(const char *prog) {
    std::fprintf(stderr, "Usage: %s [port] [baud] [--raw] [--dump-len N]\n", prog);
    std::fprintf(stderr, "Example: %s /dev/ttyUSB0 460800 --raw\n", prog);
}

} // namespace

int main(int argc, char **argv) {
    const char *port = "/dev/ttyUSB0";
    int baud = 460800;
    bool raw_dump = false;
    int dump_len = 64;
    bool port_set = false;
    bool baud_set = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (std::strcmp(argv[i], "--raw") == 0) {
            raw_dump = true;
            continue;
        }
        if (std::strcmp(argv[i], "--dump-len") == 0) {
            if (i + 1 < argc) {
                dump_len = std::atoi(argv[++i]);
            } else {
                print_usage(argv[0]);
                return 1;
            }
            continue;
        }
        if (argv[i][0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
        if (!port_set) {
            port = argv[i];
            port_set = true;
            continue;
        }
        if (!baud_set) {
            baud = std::atoi(argv[i]);
            baud_set = true;
            continue;
        }
        std::fprintf(stderr, "Unexpected argument: %s\n", argv[i]);
        print_usage(argv[0]);
        return 1;
    }
    if (baud <= 0) {
        print_usage(argv[0]);
        return 1;
    }

    int fd = open_serial(port, baud);
    if (fd < 0) {
        return 1;
    }

    std::printf("Reading %s @ %d baud (Ctrl+C to stop)\n", port, baud);
    std::fflush(stdout);

    yesense::yesense_decoder decoder;
    yesense::yis_out_data_t out{};
    unsigned char buffer[512];

    while (true) {
        ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(2000);
                continue;
            }
            std::perror("read");
            break;
        }
        if (n == 0) {
            usleep(2000);
            continue;
        }

        if (raw_dump) {
            dump_hex(buffer, static_cast<size_t>(n), dump_len);
        }

        int ret = decoder.data_proc(buffer, static_cast<unsigned int>(n), &out);
        if (ret == analysis_ok && out.content.valid_flg) {
            out.content.valid_flg = 0u;
            print_sample(out);
        }
    }

    close(fd);
    return 0;
}
