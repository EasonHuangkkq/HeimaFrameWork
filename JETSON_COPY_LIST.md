# 拷贝到 Jetson Thor 的文件清单

## 从 Mac 拷贝到 Jetson Thor 的目录结构

```
heimaFrameWork/
├── heimaSDK_copy/                    # DriverSDK 源码（已修改）
│   ├── heima_driver_sdk.h             # 修改后的头文件（16-byte struct）
│   ├── heima_driver_sdk.cpp           # 修改后的实现（kp/kd 硬编码为 0）
│   ├── CMakeLists.txt                 # 已添加 loong_driver_sdk target
│   ├── common.h
│   ├── config_xml.*
│   ├── ecat.*
│   ├── can.*
│   ├── rs485.*
│   ├── rs232.*
│   ├── yesense_*
│   ├── yesense/
│   └── tinyxml2.*
├── loong_base-main/                  # 主控制程序
│   ├── config/                       # ✅ 已创建，需要拷贝
│   │   ├── driver.ini                 # 15-motor 配置
│   │   ├── thread.ini                 # 线程配置
│   │   └── config.xml                # 15-motor EtherCAT XML
│   ├── main/driver/
│   ├── main/locomotion/
│   ├── main/interface/
│   ├── src/
│   ├── CMakeLists.txt
│   ├── tools/
│   │   ├── make.sh
│   │   ├── run_driver.sh
│   │   ├── run_locomotion.sh
│   │   └── run_interface.sh
│   └── module/
│       ├── loong_driver_sdk/
│       │   ├── joint_map.h            # 原有文件
│       │   ├── libjoint_map_a64.so     # 原有文件
│       │   └── libloong_driver_sdk_a64.so.bak  # 备份
│       └── nabo_locomotion/
└── loong_utility-main/               # 公共工具库
```

## 具体需要拷贝的文件

### 1. 必须拷贝的已修改文件

```
heimaSDK_copy/heima_driver_sdk.h
heimaSDK_copy/heima_driver_sdk.cpp
heimaSDK_copy/CMakeLists.txt
heimaSDK_copy/common.cpp
heimaSDK_copy/config_xml.cpp
heimaSDK_copy/tinyxml2.cpp
heimaSDK_copy/ecat.cpp
heimaSDK_copy/can.cpp
heimaSDK_copy/rs485.cpp
heimaSDK_copy/rs232.cpp
heimaSDK_copy/yesense_imu.cpp
heimaSDK_copy/yesense/src/yesense_decoder.cpp
heimaSDK_copy/yesense/src/yesense_std_out_decoder.cpp
heimaSDK_copy/common.h
heimaSDK_copy/config_xml.h
heimaSDK_copy/ecat.h
heimaSDK_copy/can.h
heimaSDK_copy/rs485.h
heimaSDK_copy/rs232.h
heimaSDK_copy/yesense_imu.h
heimaSDK_copy/yesense/src/yesense_decoder.h
heimaSDK_copy/yesense/src/yesense_std_out_decoder.h
heimaSDK_copy/tinyxml2.h
```

### 2. 必须拷贝的配置文件

```
loong_base-main/config/driver.ini       # ✅ 新创建
loong_base-main/config/thread.ini       # ✅ 新创建
loong_base-main/config/config.xml       # ✅ 新创建
```

### 3. 可选保留的原有文件

这些可以保留在 Jetson Thor 上，不需要从 Mac 拷贝：

```
loong_base-main/module/loong_driver_sdk/joint_map.h
loong_base-main/module/loong_driver_sdk/libjoint_map_a64.so
loong_base-main/module/nabo_locomotion/libnabo_a64.so
loong_base-main/module/nabo_locomotion/nabo.h
```

## 拷贝方法

### 方法 1: 使用 rsync（推荐）

在 Mac 上执行：

```bash
rsync -avz --progress \
  /Users/shenghuang/Downloads/heima/heimaFrameWork/heimaSDK_copy/ \
  user@jetson-thor:/path/to/heimaFrameWork/heimaSDK_copy/

rsync -avz --progress \
  /Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/ \
  user@jetson-thor:/path/to/heimaFrameWork/loong_base-main/config/
```

### 方法 2: 使用 scp

```bash
# 拷贝整个 heimaSDK_copy
scp -r /Users/shenghuang/Downloads/heima/heimaFrameWork/heimaSDK_copy \
  user@jetson-thor:/path/to/heimaFrameWork/

# 拷贝配置文件
scp /Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/* \
  user@jetson-thor:/path/to/heimaFrameWork/loong_base-main/config/
```

### 方法 3: 使用 SFTP/USB

如果网络不可用：
1. 在 Mac 上打包：
   ```bash
   tar czf heima_updates.tar.gz \
     heimaSDK_copy/ \
     loong_base-main/config/
   ```
2. 拷贝到 USB 驱动器
3. 在 Jetson Thor 上解包：
   ```bash
   tar xzf heima_updates.tar.gz -C /path/to/heimaFrameWork/
   ```

## Jetson Thor 上的后续操作

### 1. 编译 Heima SDK

```bash
cd /path/to/heimaFrameWork/heimaSDK_copy
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target loong_driver_sdk -j4
```

### 2. 替换 SDK 库

```bash
# 备份原有库
mv /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so \
   /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so.bak

# 拷贝新编译的库
cp /path/to/heimaFrameWork/heimaSDK_copy/build/libloong_driver_sdk.so \
   /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so

# 同步头文件
cp /path/to/heimaFrameWork/heimaSDK_copy/heima_driver_sdk.h \
   /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
```

### 3. 编译 loong_base-main

```bash
cd /path/to/heimaFrameWork/loong_base-main/tools
./make.sh ar
```

### 4. 运行测试

```bash
cd /path/to/heimaFrameWork/loong_base-main/tools
sudo ./run_driver.sh s
```

## 验证清单

在 Jetson Thor 上执行以下检查：

```bash
# 1. 检查配置文件存在
ls -la /path/to/heimaFrameWork/loong_base-main/config/
# 应该看到: driver.ini, thread.ini, config.xml

# 2. 检查头文件版本
grep -A 5 "struct motorTargetStruct" \
  /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
# 应该只有 pos, vel, tor, enabled (16 bytes total)

# 3. 检查 SDK 库已替换
ls -lh /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so
# 应该是新编译的库（时间戳是最近的）

# 4. 检查可执行文件生成
ls -lh /path/to/heimaFrameWork/loong_base-main/nabo_output/loong_driver_a64
```

## 常见问题

### Q: 如果 Jetson Thor 上已经有 loong_base-main 代码怎么办？

A: 只需要拷贝修改的部分：

```bash
# 1. 拷贝 heimaSDK_copy（完整替换）
scp -r heimaSDK_copy user@jetson-thor:/path/to/heimaFrameWork/

# 2. 只拷贝配置文件
scp loong_base-main/config/* user@jetson-thor:/path/to/heimaFrameWork/loong_base-main/config/

# 3. 拷贝头文件
scp heimaSDK_copy/heima_driver_sdk.h \
  user@jetson-thor:/path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/
```

然后在 Jetson Thor 上编译和替换库。

### Q: 编译失败提示找不到 libethercat.so？

A: 确保已安装 EtherCAT master：

```bash
# 检查库是否存在
ls -l /usr/local/lib/libethercat.so
ls -l /usr/lib/libethercat.so

# 如果不存在，需要安装
sudo apt-get update
sudo apt-get install ethercat-master
```

### Q: 编译后运行时出现段错误？

A: 可能是 ABI 不兼容。检查：

```bash
# 确保 motorTargetStruct 大小正确
objdump -t -C /path/to/heimaFrameWork/heimaSDK_copy/build/libloong_driver_sdk.so | grep motorTargetStruct
```

应该在 loong_base-main 的链接时看到 16-byte 版本。
