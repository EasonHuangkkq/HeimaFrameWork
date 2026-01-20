# Python 可视化（motor/IMU 状态）

这里提供一个最小可用的方案：把 SDK 封装成 `libheima_sdk_c.so`（C ABI，方便 `ctypes`），然后用 Python 周期读取每个电机的 `status_word/error_code/pos/vel/tor/temp` 和 IMU 的 `rpy/gyr/acc`。

## 1) 编译 `libheima_sdk_c.so`

### 方式 A：CMake（推荐）

```bash
cd heimaSDK_copy
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target heima_sdk_c
```

产物：`heimaSDK_copy/build/libheima_sdk_c.so`

### 方式 B：g++ 单条命令

```bash
cd heimaSDK_copy
mkdir -p build
g++ -std=c++20 -O2 -fPIC -shared -pthread -DENABLE_RS232 \
  heima_sdk_c_api.cpp heima_driver_sdk.cpp ecat.cpp \
  common.cpp config_xml.cpp tinyxml2.cpp \
  rs232.cpp yesense_imu.cpp \
  yesense/src/yesense_decoder.cpp yesense/src/yesense_std_out_decoder.cpp \
  rs485.cpp can.cpp \
  -I. -I/usr/local/include -L/usr/local/lib -lethercat -lm \
  -o build/libheima_sdk_c.so
```

## 2) Python 运行（终端表格刷新）

```bash
cd heimaSDK_copy
sudo python3 python/visualize_status.py --config config.xml --lib build/libheima_sdk_c.so --hz 10
```

说明：
- `--hz` 控制刷新频率；`status_word==0xffff` 会显示为 `OFF`（inactive）。
- limb 分组来自 `config.xml` 里的 `<Motors><Motor limb="..." motor="..." alias="...">`。
- IMU 依赖 `config.xml` 里的 `<IMU device="..." baudrate="..." type="YeSense"/>`，且库是按 `ENABLE_RS232` 编译的；IMU 打不开会导致 SDK 初始化失败。

