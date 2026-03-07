# Whole Body Smoke Test Data Flow

This document traces runtime flow of `whole_body_smoke_test.cpp` when `config.xml`
contains motor mapping (33 motors across 6 limbs) with optional IMU data display.

## 0) Preconditions and build flags

- config.xml must contain proper motor mapping for 6 limbs:
  - limb=0 (Left Leg)
  - limb=1 (Right Leg)
  - limb=2 (Left Arm)
  - limb=3 (Right Arm)
  - limb=4 (Waist)
  - limb=5 (Neck)
  - Total motors should be 33 (distributed across limbs)
- IMU device (optional) must be configured in config.xml:
  - `<IMU device="/dev/ttyACM0" baudrate="460800" type="YeSense"/>`
- Build flags (see header comments in source file):
  - `-std=c++20` - Modern C++ features
  - `-O2` - Optimization level 2
  - `-pthread` - Threading support
  - `-lethercat` - Link EtherCAT library
  - `-lm` - Link math library
    **IMU REQUIREMENT**: To enable IMU data display, must define `ENABLE_RS232` at compile time:
  - Add `-DENABLE_RS232` to compilation flags
  - Example: `cmake --build build -DENABLE_RS232`
  - Or modify `CMakeLists.txt` to add: `target_compile_definitions(whole_body_smoke_test PRIVATE ENABLE_RS232)`
  - **IMPORTANT**: If `ENABLE_RS232` is NOT defined, `sdk.getIMU()` will do nothing and no IMU data will be displayed!
  - This is controlled by `#ifdef ENABLE_RS232` in `rs232.h` which stubs out the IMU class
  - See `heimaSDK/rs232.h:24-78` and `heimaSDK/heima_driver_sdk.cpp:760`
  - **See section 7 for troubleshooting IMU issues**
  - **Solution**: Recompile with `-DENABLE_RS232` flag:
    ```bash
    cmake --build build -DENABLE_RS232
    ```
    - Or manually modify `CMakeLists.txt` to include compile definition:
      `target_compile_definitions(whole_body_smoke_test PRIVATE ENABLE_RS232)`

## 14) Output examples

### Example 1: Monitor all motors + IMU

```bash
sudo ./build/whole_body_smoke_test --config config.xml --all --imu
```

Output:

```
Config: config.xml total_motors=33
Selection: all_limbs count=33
Mode: 8 period_us: 1000 enable: no seconds: 10
IMU enabled: yes
[m1] alias=1 idx=0 pos=0.0123 vel=0.001 tor=0.0 sw=0x37
...
[IMU]
  RP Y [rad]: 0.0231 -0.0456 1.234
  Gyr [rad/s]: 0.0012 0.0023 -0.0045
  Acc [m/s²]: 0.123 0.234 9.810
```

### Example 2: Test left arm with sine wave + IMU

```bash
sudo ./build/whole_body_smoke_test --config config.xml --limb 2 --enable --joint 0 --amp 0.05 --freq 0.2 --imu
```

Output:

```
[RUN] tick=0 motor_count=7
  [m9] alias=9 idx=8 pos=0.0123 vel=0.001 tor=0.0 sw=0x37 cmd_pos=0.0123 en=1
  ...
[IMU]
  RP Y [rad]: 0.0231 -0.0456 1.234
  Gyr [rad/s]: 0.0012 0.0023 -0.0045
  Acc [m/s²]: 0.123 0.234 9.810
```

### Example 3: Sequential test of all motors + IMU (with ENABLE_RS232 defined)

```bash
# Enable IMU support by defining compile flag
cmake --build build -DENABLE_RS232
sudo ./build/whole_body_smoke_test --config config.xml --all --enable --sequential --joint 0 --amp 0.05 --freq 0.2 --imu
```

Output:

```
Config: config.xml total_motors=33
Selection: all_limbs count=33
Mode: 8 period_us: 1000 enable: yes seconds: 10
Sequential: yes delay_ms: 1000
IMU enabled: yes

=== Testing joint 0 (alias=1) ===
[SEQ] test_j=0 tick=0 m1 pos=0.0123 cmd=0.0123
[SEQ-IMU]
  RP Y [rad]: 0.0231 -0.0456 1.234
  Gyr [rad/s]: 0.0012 0.0023 -0.0045
  Acc [m/s²]: 0.123 0.234 9.810
...
=== Testing joint 1 (alias=2) ===
[SEQ] test_j=1 tick=0 m2 pos=-0.0456 cmd=-0.0456
[SEQ-IMU]
  RP Y [rad]: 0.0231 -0.0456 1.234
...
```

### Example 4: Test specific motors by alias + IMU

```bash
sudo ./build/whole_body_smoke_test --config config.xml --aliases 1,2,3,4,5,6 --enable --imu
```

Output:

```
Config: config.xml total_motors=33
Selection: aliases=1,2,3,4,5,6 count=6
Mode: 8 period_us: 1000 enable: yes seconds: 10
IMU enabled: yes
[m1] alias=1 idx=0 pos=0.0123 vel=0.001 tor=0.0 sw=0x37 cmd_pos=0.0123 en=1
[m2] alias=2 idx=1 pos=-0.0456 vel=-0.001 tor=0.1 sw=0x37 cmd_pos=-0.0456 en=1
...
[IMU]
  RP Y [rad]: 0.0231 -0.0456 1.234
  Gyr [rad/s]: 0.0012 0.0023 -0.0045
  Acc [m/s²]: 0.123 0.234 9.810
```

8. Motor alias out of range:
   - `alias > motorCount` → Error: "Motor alias out of range"
   - `heimaSDK_copy/whole_body_smoke_test.cpp:461-463`

g++ -std=c++20 -O2 -pthread -DENABLE_RS232 \
 whole_body_smoke_test.cpp heima_driver_sdk.cpp ecat.cpp \
 common.cpp config_xml.cpp tinyxml2.cpp \
 rs232.cpp yesense_imu.cpp \
 yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp \
 rs485.cpp can.cpp \
 -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
 -o build/whole_body_smoke_test
