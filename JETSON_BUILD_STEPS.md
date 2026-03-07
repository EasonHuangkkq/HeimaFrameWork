# Jetson Thor 上的编译步骤

## 前提条件
- Jetson Thor 上已安装 EtherCAT master 库 (`libethercat.so`)
- 已安装 CMake 3.10+
- 已安装 C++20 支持的编译器 (gcc/g++)

## 步骤 1: 拷贝代码到 Jetson Thor

需要从 Mac 拷贝以下目录到 Jetson Thor：

```
heimaFrameWork/
├── heimaSDK_copy/          # 修改后的 DriverSDK 源码
├── loong_base-main/        # 主控制程序
│   ├── config/             # 已创建的配置文件 (driver.ini, thread.ini, config.xml)
│   └── module/loong_driver_sdk/  # 原有的预编译库
└── loong_utility-main/    # 公共工具库
```

## 步骤 2: 编译 Heima DriverSDK

```bash
cd /path/to/heimaFrameWork/heimaSDK_copy

# 清理之前的构建（如果有）
rm -rf build build_a64

# 配置 CMake（Jetson Thor 原生编译，不需要 cross-compile）
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 编译 loong_driver_sdk 目标（这个是 loong_base-main 需要的）
cmake --build . --target loong_driver_sdk -j4

# 验证生成的库
ls -lh libloong_driver_sdk.so
```

生成的库在：`heimaSDK_copy/build/libloong_driver_sdk.so`

## 步骤 3: 替换 loong_base-main 的 SDK 库

```bash
cd /path/to/heimaFrameWork

# 备份原来的库（可选）
mv loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so \
   loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so.bak

# 拷贝新编译的库
cp heimaSDK_copy/build/libloong_driver_sdk.so \
   loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so

# 创建软链接（可选，保持兼容性）
cd loong_base-main/module/loong_driver_sdk
ln -sf libloong_driver_sdk_a64.so libloong_driver_sdk.so
```

**重要**: 拷贝 `heimaSDK_copy/heima_driver_sdk.h` 到 module 目录，确保头文件版本一致：

```bash
cp /path/to/heimaFrameWork/heimaSDK_copy/heima_driver_sdk.h \
   /path/to/heimaFrameWork/loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
```

## 步骤 4: 编译 loong_base-main

```bash
cd /path/to/heimaFrameWork/loong_base-main/tools

# 编译 aarch64 robot 版本
./make.sh ar
```

这会生成 `nabo_output/loong_driver_a64` 可执行文件。

## 步骤 5: 运行测试

```bash
cd /path/to/heimaFrameWork/loong_base-main/tools

# 运行 driver 任务（使用 config/driver.ini 中配置的 15 个电机）
sudo ./run_driver.sh s
```

`run_driver.sh s` 的 `s` 参数表示使用 `config/` 目录下的配置。

## 故障排查

### 如果 CMake 找不到 ethercat 库

```bash
# 检查 libethercat.so 是否存在
ls -l /usr/local/lib/libethercat.so
ls -l /usr/lib/libethercat.so

# 如果在非标准路径，修改 heimaSDK_copy/CMakeLists.txt 中的路径
```

### 如果缺少 C++20 编译器支持

```bash
# 检查 g++ 版本
g++ --version

# Jetson 上可能需要安装新版本
sudo apt-get update
sudo apt-get install g++-10
```

### 如果编译 loong_base-main 失败

确保 `loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h` 与你编译的库 ABI 兼容：

```bash
# 检查头文件中的 motorTargetStruct 大小应该是 16 字节
grep -A 5 "struct motorTargetStruct" loong_driver_sdk.h
```

应该看到：
```cpp
struct motorTargetStruct{
    float pos, vel, tor;   // 12 bytes
    int enabled;            // 4 bytes
};                        // Total: 16 bytes
```

## 验证清单

- [ ] Heima SDK 编译成功：`libloong_driver_sdk.so` 生成
- [ ] 库已拷贝到 `loong_base-main/module/loong_driver_sdk/`
- [ ] 头文件已同步：`heima_driver_sdk.h` 拷贝到 module 目录
- [ ] loong_base-main 编译成功：`loong_driver_a64` 生成
- [ ] 运行 `run_driver.sh s` 无错误启动
- [ ] 日志显示检测到 15 个电机：`[init] dofAll = 15`
