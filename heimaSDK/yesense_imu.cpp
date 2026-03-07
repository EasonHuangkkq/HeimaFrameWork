/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#include "yesense_imu.h"
#include <cstring>

namespace DriverSDK{
YesenseImuDecoder::YesenseImuDecoder(){
    std::memset(&result, 0, sizeof(result));
}

bool YesenseImuDecoder::feed(unsigned char const* data, size_t len, YesenseImuFrame& out){
    if(data == nullptr || len == 0){
        return false;
    }
    int ret = decoder.data_proc(const_cast<unsigned char*>(data), (unsigned int)len, &result);
    if(ret != analysis_ok){
        return false;
    }
    std::memset(&out, 0, sizeof(out));
    out.valid = result.content.valid_flg ? 1 : 0;
    out.tid = result.tid;
    if(result.content.euler){
        out.rpy[0] = result.euler.roll * Pi / 180.0f;
        out.rpy[1] = result.euler.pitch * Pi / 180.0f;
        out.rpy[2] = result.euler.yaw * Pi / 180.0f;
    }
    if(result.content.gyro){
        out.gyr[0] = result.gyro.x * Pi / 180.0f;
        out.gyr[1] = result.gyro.y * Pi / 180.0f;
        out.gyr[2] = result.gyro.z * Pi / 180.0f;
    }
    if(result.content.acc){
        out.acc[0] = result.acc.x;
        out.acc[1] = result.acc.y;
        out.acc[2] = result.acc.z;
    }
    return true;
}

static YesenseImuFrame const* frameFromSwap(SwapList const* txSwap){
    return reinterpret_cast<YesenseImuFrame const*>(txSwap->nodePtr.load()->memPtr);
}

float rpy0yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->rpy[0];
}

float rpy1yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->rpy[1];
}

float rpy2yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->rpy[2];
}

float gyr0yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->gyr[0];
}

float gyr1yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->gyr[1];
}

float gyr2yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->gyr[2];
}

float acc0yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->acc[0];
}

float acc1yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->acc[1];
}

float acc2yesense(SwapList const* txSwap){
    return frameFromSwap(txSwap)->acc[2];
}
}
