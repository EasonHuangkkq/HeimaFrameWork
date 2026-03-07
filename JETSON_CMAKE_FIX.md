# Jetson Thor 上 CMake 编译步骤（解决缓存冲突）

## 问题：CMake 缓存冲突

错误信息说明：
- 之前在 `/home/heima-thor/eason/heimaSDK/build` 运行过 cmake
- 现在想在 `/home/thor-pc/heimaFrameWork/heimaSDK_copy/build` 运行
- CMake 检测到源路径不匹配，拒绝继续

## 解决方法

### 方法 1：删除 build 目录重新构建（推荐）

```bash
# 删除 build 目录（包括 CMakeCache.txt）
cd /home/thor-pc/heimaFrameWork/heimaSDK_copy
rm -rf build

# 重新创建并配置
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build . --target loong_driver_sdk -j4
```

### 方法 2：使用全新构建目录

```bash
# 使用不同的构建目录名
cd /home/thor-pc/heimaFrameWork/heimaSDK_copy
mkdir -p build_new && cd build_new
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build . --target loong_driver_sdk -j4
```

### 方法 3：清理所有构建缓存（彻底重置）

```bash
cd /home/thor-pc/heimaFrameWork/heimaSDK_copy

# 删除所有可能的构建目录和缓存
rm -rf build build_* CMakeCache.txt CMakeFiles
find . -name "CMakeCache.txt" -delete
find . -type d -name "CMakeFiles" -exec rm -rf {} + 2>/dev/null || true

# 重新构建
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target loong_driver_sdk -j4
```

## 推荐的完整编译流程

```bash
# 1. 进入源码目录
cd /home/thor-pc/heimaFrameWork/heimaSDK_copy

# 2. 清理旧的构建
rm -rf build

# 3. 创建新构建目录并配置
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release

# 4. 编译 loong_driver_sdk 库
cmake --build . --target loong_driver_sdk -j4

# 5. 验证生成的库
ls -lh libloong_driver_sdk.so
```

## 预期输出

如果成功，应该看到：

```
-- The CXX compiler identification is GNU ... (或 aarch64-linux-gnu-g++)
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Found EtherCAT Lib: /usr/local/lib/libethercat.so
-- Configuring done
-- Generating done
[ 20%] Building CXX object CMakeFiles/...
[ 40%] Building CXX object CMakeFiles/...
[ 60%] Building CXX object CMakeFiles/...
[ 80%] Building CXX object CMakeFiles/...
[100%] Linking CXX shared library libloong_driver_sdk.so
```

## 后续步骤

编译成功后：

```bash
# 1. 拷贝库到 loong_base-main
cd /home/thor-pc/heimaFrameWork
cp heimaSDK_copy/build/libloong_driver_sdk.so \
   loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so

# 2. 拷贝头文件
cp heimaSDK_copy/heima_driver_sdk.h \
   loong_base-main/module/loong_driver_sdk/

# 3. 编译 loong_base-main
cd loong_base-main/tools
./make.sh ar

# 4. 运行测试
sudo ./run_driver.sh s
```

## 避免再次遇到此问题

**永远使用独立的 build 目录**：
- ✅ `/home/thor-pc/heimaFrameWork/heimaSDK_copy/build`
- ❌ 不要在 `/home/heima-thor/...` 和 `/home/thor-pc/...` 之间混用
- ✅ 每次重新配置前先 `rm -rf build`
