/* Stub CAN interface for builds that do not require CAN support.
 * Provides minimal definitions to satisfy references in the DriverSDK
 * without introducing external dependencies.
 */

#pragma once

#include "common.h"
#include <map>
#include <pthread.h>
#include <vector>

namespace DriverSDK{
// Placeholder for driver parameters carried alongside CAN devices.
struct DriverParameters{};

class CAN{
public:
    static int CANHAL;
    static pthread_t rxPth, txPth, txPth_;
    static std::map<std::string, DriverParameters> type2parameters;
    static int* alias2masterID_;
    static unsigned short* alias2status;
    static DriverParameters** alias2parameters;
    static int orderSlaveID2alias[8][16];

    int order;
    SwapList* rxSwap;
    SwapList* txSwap;

    CAN(int order, char const* device);
    int config();
    static int run(std::vector<CAN>& cans);
    ~CAN();
};
}
