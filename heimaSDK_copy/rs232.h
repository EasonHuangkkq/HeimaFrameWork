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
#include <atomic>
#include <pthread.h>

#ifdef ENABLE_RS232

namespace DriverSDK{
using validationFunction = bool (*)(unsigned char const*);
using parseFunction = float (*)(SwapList const*);
class YesenseImuDecoder;

struct ChainNode{
    int nr;
    ChainNode* previous, * next;
};

class RS232{
public:
    char* device;
    int baudrate, frameLength;
    unsigned char header0, header1;
    std::atomic<ChainNode*> ptr;
    SwapList* txSwap;
    pthread_t pth;
    validationFunction valid;
    parseFunction rpy0, rpy1, rpy2, gyr0, gyr1, gyr2, acc0, acc1, acc2;
    bool useYesense;
    YesenseImuDecoder* yesenseDecoder;
    RS232(char const* device, int const baudrate, char const* type);
    static void cleanup(void* arg);
    static void* recv(void* arg);
    int run();
    ~RS232();
};
}

#else  // ENABLE_RS232

namespace DriverSDK{
using validationFunction = bool (*)(unsigned char const*);
using parseFunction = float (*)(SwapList const*);

class RS232{
public:
    char* device = nullptr;
    int baudrate = 0;
    int frameLength = 0;
    unsigned char header0 = 0, header1 = 0;
    SwapList* txSwap = nullptr;
    pthread_t pth = 0;
    validationFunction valid = nullptr;
    parseFunction rpy0 = nullptr, rpy1 = nullptr, rpy2 = nullptr, gyr0 = nullptr, gyr1 = nullptr, gyr2 = nullptr, acc0 = nullptr, acc1 = nullptr, acc2 = nullptr;
    RS232(char const*, int, char const*){}
    static void cleanup(void*){}
    static void* recv(void*){ return nullptr; }
    int run(){ return 0; }
    ~RS232() = default;
};
}

#endif
