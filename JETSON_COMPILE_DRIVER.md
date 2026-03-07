# 编译 loong_base-main driver task

## 步骤 1: 拷贝 SDK 库到 loong_base-main

```bash
# 在 Jetson Thor 上执行：
cd /home/thor-pc/heimaFrameWork

# 备份原有库
cp loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so \
   loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so.bak

# 拷贝新编译的库
cp heimaSDK_copy/build_new/libloong_driver_sdk.so \
   loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so

# 拷贝头文件（确保版本一致）
cp heimaSDK_copy/heima_driver_sdk.h \
   loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
```

## 步骤 2: 编译 loong_base-main

```bash
cd /home/thor-pc/heimaFrameWork/loong_base-main/tools

# 编译 aarch64 robot 版本
./make.sh ar
```

这会在 `loong_base-main/nabo_output/` 生成 `loong_driver_a64` 可执行文件。

## 步骤 3: 运行 driver task

```bash
cd /home/thor-pc/heimaFrameWork/loong_base-main/tools

# 运行 driver task（使用 config/ 目录下的配置）
sudo ./run_driver.sh s
```

## 预期输出

运行 `./run_driver.sh s` 后，应该看到：

```
[init] dofAll = 15
[init] 检测到 15 个驱动器
[init] ecat config done
[init] ecat run started
[init] driver 链接成功
```

## 验证清单

- [ ] SDK 库已拷贝：`libloong_driver_sdk_a64.so` 存在
- [ ] 头文件已同步：`loong_driver_sdk.h` 与编译的 SDK 匹配
- [ ] loong_base-main 编译成功：`nabo_output/loong_driver_a64` 生成
- [ ] 运行无错误：`sudo ./run_driver.sh s` 正常启动
- [ ] 检测到 15 个电机：日志显示 `[init] dofAll = 15`

## 常见问题

### Q: 编译 loong_base-main 时提示找不到 loong_driver_sdk.h？

A: 确保头文件已拷贝：
```bash
ls -la loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
```

### Q: 链接时提示符号未定义？

A: 检查 SDK 库是否正确：
```bash
nm -D loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so | grep DriverSDK::instance
```

应该能看到 `_ZN9DriverSDK8instanceEv`。

### Q: 运行时找不到 libloong_driver_sdk_a64.so？

A: 确保在正确目录运行，并检查环境：
```bash
# 在 loong_base-main/ 目录下运行
cd /home/thor-pc/heimaFrameWork/loong_base-main
ls -lh module/loong_driver_sdk/libloong_driver_sdk_a64.so

# 检查运行时库路径
echo $LD_LIBRARY_PATH
```

### Q: 启动后立即崩溃？

A: 可能是 ABI 不兼容。检查头文件中的 struct 大小：
```bash
grep -A 5 "struct motorTargetStruct" \
   loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h
```

应该只看到 4 个字段：`pos, vel, tor, enabled`（16 字节）。
