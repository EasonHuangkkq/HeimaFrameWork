# Yesense SDK Test Data Flow (from config.xml IMU entry)

This document traces the runtime flow of `yesense_sdk_test.cpp` when `config.xml`
contains the IMU entry:

`<IMU device="/dev/ttyACM0" baudrate="460800" type="YeSense"/>`

It is organized as a step-by-step chain from configuration to printed output,
with concrete code points.

## 0) Preconditions and build flags

- `ENABLE_RS232` must be defined at build time. Otherwise IMU is stubbed out
  and `getIMU()` does nothing. See `heimaSDK/heima_driver_sdk.cpp:760`.

## 1) Entry point: yesense_sdk_test.cpp

1. The test program picks a config path (default `config.xml`) and installs
   SIGINT/SIGTERM handlers to exit cleanly. `heimaSDK/yesense_sdk_test.cpp:31`.
2. It initializes the SDK: `sdk.init(configPath.c_str())`.
   `heimaSDK/yesense_sdk_test.cpp:40`, `heimaSDK/heima_driver_sdk.cpp:667`.
3. It loops at ~100 Hz:
   - Calls `sdk.getIMU(imu)` to fetch the latest IMU frame.
   - Prints rpy/gyr/acc once every 100 ticks (about 1 Hz).
   `heimaSDK/yesense_sdk_test.cpp:43`.

## 2) Config parsing: IMU attributes

1. `DriverSDK::init()` forwards into `imp.init(xmlFile)`.
   `heimaSDK/heima_driver_sdk.cpp:667`.
2. `imp.init()` constructs `ConfigXML` and parses the XML tree.
   `heimaSDK/heima_driver_sdk.cpp:157`.
3. `ConfigXML::imuAttribute()` and `ConfigXML::imuBaudrate()` read the IMU
   attributes directly from `<Config><IMU .../>`.
   `heimaSDK/config_xml.cpp:110`.
4. The actual IMU entry in your `config.xml` is at:
   `heimaSDK/config.xml:105`.

## 3) RS232 object creation and thread start

1. Under `ENABLE_RS232`, `imp.init()` creates the RS232 instance:
   `imu = new RS232(device, baudrate, type);`
   `heimaSDK/heima_driver_sdk.cpp:285`.
2. Then `imu->run()` spawns the receive thread and `pthread_detach()` is called
   on that thread.
   `heimaSDK/heima_driver_sdk.cpp:287`, `heimaSDK/rs232.cpp:385`.

## 4) RS232 constructor: YeSense specialization

When `type == "YeSense"`:

1. `frameLength = sizeof(YesenseImuFrame)`.
2. Function pointers `rpy0/gyr0/acc0/...` are bound to YeSense accessors.
3. `useYesense = true` and `yesenseDecoder = new YesenseImuDecoder()`.
4. `txSwap = new SwapList(frameLength)` creates a 3-node buffer for frames.

See `heimaSDK/rs232.cpp:162` and `heimaSDK/yesense_imu.h:26`.

## 5) RX thread: serial read and decode (YeSense branch)

The recv thread (`RS232::recv`) does the following for YeSense:

1. Selects the termios baud constant based on `imu->baudrate`.
2. Opens the device (read-only) and configures the port:
   - 8N1, no flow control
   - `VTIME = 1` (100 ms timeout), `VMIN = 0`
3. Continuously reads into `buff[512]`.
4. For each read, it calls `yesenseDecoder->feed(buff, n, frame)`.
5. If `feed()` returns true:
   - `frame` is copied into `txSwap->nodePtr.load()->next->memPtr`
   - `txSwap->advanceNodePtr()` publishes the new frame

See `heimaSDK/rs232.cpp:239`.

## 6) YesenseImuDecoder: parsing and frame layout

`YesenseImuDecoder::feed()` uses the vendor decoder to parse the byte stream:

1. Calls `decoder.data_proc(...)` to update `result`.
2. If parsing succeeds:
   - `valid` is set from `result.content.valid_flg`.
   - `tid` is set from `result.tid`.
   - `rpy` is filled from `result.euler` (degrees -> radians).
   - `gyr` is filled from `result.gyro` (degrees/s -> radians/s).
   - `acc` is filled from `result.acc` (no conversion).

See `heimaSDK/yesense_imu.cpp:26` and frame layout in
`heimaSDK/yesense_imu.h:26`.

The in-memory frame that gets copied into `txSwap` is:

```
struct YesenseImuFrame {
    float rpy[3];
    float gyr[3];
    float acc[3];
    unsigned short tid;
    unsigned char valid;
};
```

## 7) SwapList buffering semantics (IMU)

`txSwap` is a `SwapList` (3 nodes). For YeSense:

- Writer thread copies into `nodePtr->next->memPtr` then advances.
- Reader uses `nodePtr->memPtr` as the current frame.

This prevents readers from observing partially written frames without
explicit locking. The only atomic operation is the pointer swap inside
`advanceNodePtr()`.

See `heimaSDK/common.cpp:104` for `advanceNodePtr()` implementation.

## 8) SDK readout: DriverSDK::getIMU

`yesense_sdk_test.cpp` calls `sdk.getIMU(imu)`, which does:

1. Reads `rpy` from `imp.imu->rpy0/1/2(txSwap)` and only accepts values
   in the range `(-4.0, 4.0)` radians.
2. Reads `gyr` from `imp.imu->gyr0/1/2(txSwap)` and only accepts values
   in the range `(-40.0, 40.0)` rad/s.
3. Reads `acc` from `imp.imu->acc0/1/2(txSwap)` without filtering.

See `heimaSDK/heima_driver_sdk.cpp:760`.

Note: If a value is out of range, that field is not updated, so the
output can hold the previous value for that axis.

## 9) Output in yesense_sdk_test.cpp

`yesense_sdk_test.cpp` prints the IMU values every 100 iterations and sleeps
10 ms between iterations:

- Output cadence: ~1 Hz.
- Read cadence: ~100 Hz.

See `heimaSDK/yesense_sdk_test.cpp:45`.

## 10) Failure points and behavior

- If the IMU device cannot be opened or baudrate is invalid, the recv
  thread prints an error and exits the process. `heimaSDK/rs232.cpp:251`.
- If `ENABLE_RS232` is not defined, IMU is disabled and `getIMU()` does nothing.
  `heimaSDK/heima_driver_sdk.cpp:760`.
- `YesenseImuFrame.valid` is populated but not checked by `getIMU()`.
  `heimaSDK/yesense_imu.cpp:35`.

## 11) One-line flow summary

`config.xml` IMU entry -> `ConfigXML` -> `RS232` (YeSense mode) -> recv thread
reads serial -> `YesenseImuDecoder` -> `txSwap` current frame -> `getIMU()` ->
`yesense_sdk_test.cpp` prints.
