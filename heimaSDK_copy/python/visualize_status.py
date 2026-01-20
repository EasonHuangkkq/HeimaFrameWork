#!/usr/bin/env python3

import argparse
import ctypes
import os
import sys
import time
import xml.etree.ElementTree as ET


class HeimaImu(ctypes.Structure):
    _fields_ = [
        ("rpy", ctypes.c_float * 3),
        ("gyr", ctypes.c_float * 3),
        ("acc", ctypes.c_float * 3),
    ]


class HeimaMotorActual(ctypes.Structure):
    _fields_ = [
        ("pos", ctypes.c_float),
        ("vel", ctypes.c_float),
        ("tor", ctypes.c_float),
        ("temp", ctypes.c_int16),
        ("status_word", ctypes.c_uint16),
        ("error_code", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16),
    ]


def parse_limb_alias_map(config_xml_path: str):
    tree = ET.parse(config_xml_path)
    root = tree.getroot()
    limb_to_alias = {}
    for motor in root.findall("./Motors/Motor"):
        limb = int(motor.get("limb", "-1"))
        motor_idx = int(motor.get("motor", "-1"))
        alias = int(motor.get("alias", "-1"))
        if limb < 0 or motor_idx < 0 or alias < 1:
            continue
        limb_to_alias.setdefault(limb, []).append((motor_idx, alias))
    for limb, items in limb_to_alias.items():
        items.sort(key=lambda x: x[0])
    return limb_to_alias


def decode_state(status_word: int) -> str:
    if status_word == 0xFFFF:
        return "OFF"
    sw = status_word & 0x007F
    if sw == 0x37:
        return "OE"
    if sw == 0x33:
        return "SON"
    if sw == 0x31:
        return "RDY"
    if sw == 0x08:
        return "FLT"
    return f"{sw:02x}"


def clear_screen():
    sys.stdout.write("\x1b[2J\x1b[H")


def main():
    parser = argparse.ArgumentParser(description="Heima SDK motor+IMU status viewer (ctypes)")
    parser.add_argument("--config", default="config.xml", help="Path to config.xml")
    parser.add_argument("--lib", default="", help="Path to libheima_sdk_c.so (default: ./build/libheima_sdk_c.so)")
    parser.add_argument("--hz", type=float, default=10.0, help="Refresh rate (Hz)")
    args = parser.parse_args()

    lib_path = args.lib
    if not lib_path:
        candidate = os.path.join(os.path.dirname(os.path.dirname(__file__)), "build", "libheima_sdk_c.so")
        lib_path = candidate if os.path.exists(candidate) else "libheima_sdk_c.so"

    lib = ctypes.CDLL(lib_path)
    lib.heima_sdk_init.argtypes = [ctypes.c_char_p]
    lib.heima_sdk_init.restype = ctypes.c_int
    lib.heima_sdk_get_total_motor_nr.argtypes = []
    lib.heima_sdk_get_total_motor_nr.restype = ctypes.c_int
    lib.heima_sdk_get_motor_actuals.argtypes = [ctypes.POINTER(HeimaMotorActual), ctypes.c_int]
    lib.heima_sdk_get_motor_actuals.restype = ctypes.c_int
    lib.heima_sdk_get_imu.argtypes = [ctypes.POINTER(HeimaImu)]
    lib.heima_sdk_get_imu.restype = ctypes.c_int
    lib.heima_sdk_version.argtypes = []
    lib.heima_sdk_version.restype = ctypes.c_char_p

    ret = lib.heima_sdk_init(args.config.encode("utf-8"))
    if ret != 0:
        raise SystemExit(f"heima_sdk_init failed: {ret}")

    motor_nr = lib.heima_sdk_get_total_motor_nr()
    if motor_nr <= 0:
        raise SystemExit(f"invalid motor_nr: {motor_nr}")

    limb_to_alias = parse_limb_alias_map(args.config)
    limb_names = {
        0: "L_LEG",
        1: "R_LEG",
        2: "L_ARM",
        3: "R_ARM",
        4: "WAIST",
        5: "NECK",
    }

    actuals = (HeimaMotorActual * motor_nr)()
    imu = HeimaImu()

    period = 1.0 / max(args.hz, 0.1)
    last = time.time()
    while True:
        lib.heima_sdk_get_motor_actuals(actuals, motor_nr)
        lib.heima_sdk_get_imu(ctypes.byref(imu))

        now = time.time()
        dt = now - last
        last = now

        clear_screen()
        version = (lib.heima_sdk_version() or b"").decode("utf-8", errors="ignore")
        sys.stdout.write(f"heima_sdk_c: {version} | motors: {motor_nr} | dt: {dt*1000.0:.1f} ms\n\n")
        sys.stdout.write("limb  m#  alias  st   status    err     pos(rad)    vel(rad/s)     tor(Nm)  temp\n")
        sys.stdout.write("----  --  -----  ---  --------  ------  ----------  -----------  ---------  ----\n")

        for limb in sorted(limb_to_alias.keys()):
            name = limb_names.get(limb, f"LIMB{limb}")
            for motor_idx, alias in limb_to_alias[limb]:
                if not (1 <= alias <= motor_nr):
                    continue
                a = actuals[alias - 1]
                st = decode_state(int(a.status_word))
                sys.stdout.write(
                    f"{name:4s}  {motor_idx:2d}  {alias:5d}  {st:>3s}  "
                    f"0x{int(a.status_word):04x}  0x{int(a.error_code):04x}  "
                    f"{a.pos:+10.4f}  {a.vel:+11.4f}  {a.tor:+9.3f}  {int(a.temp):4d}\n"
                )

        sys.stdout.write("\n")
        sys.stdout.write(
            "IMU rpy(rad): [{:+.3f}, {:+.3f}, {:+.3f}]  "
            "gyr(rad/s): [{:+.3f}, {:+.3f}, {:+.3f}]  "
            "acc(m/s^2): [{:+.3f}, {:+.3f}, {:+.3f}]\n".format(
                imu.rpy[0],
                imu.rpy[1],
                imu.rpy[2],
                imu.gyr[0],
                imu.gyr[1],
                imu.gyr[2],
                imu.acc[0],
                imu.acc[1],
                imu.acc[2],
            )
        )
        sys.stdout.flush()

        sleep_time = period - (time.time() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()

