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

#pragma once

#include "common.h"
#include <cstddef>
#include "yesense/src/yesense_decoder.h"
#include "yesense/src/yesense_decoder_comm.h"

namespace DriverSDK{
struct YesenseImuFrame{
    float rpy[3];
    float gyr[3];
    float acc[3];
    unsigned short tid;
    unsigned char valid;
};

class YesenseImuDecoder{
public:
    YesenseImuDecoder();
    bool feed(unsigned char const* data, size_t len, YesenseImuFrame& out);
private:
    yesense::yesense_decoder decoder;
    yesense::yis_out_data_t result{};
};

float rpy0yesense(SwapList const* txSwap);
float rpy1yesense(SwapList const* txSwap);
float rpy2yesense(SwapList const* txSwap);
float gyr0yesense(SwapList const* txSwap);
float gyr1yesense(SwapList const* txSwap);
float gyr2yesense(SwapList const* txSwap);
float acc0yesense(SwapList const* txSwap);
float acc1yesense(SwapList const* txSwap);
float acc2yesense(SwapList const* txSwap);
}
