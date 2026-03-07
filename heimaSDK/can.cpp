#include "can.h"

namespace DriverSDK{
int CAN::CANHAL = 0;
pthread_t CAN::rxPth = 0;
pthread_t CAN::txPth = 0;
pthread_t CAN::txPth_ = 0;
std::map<std::string, DriverParameters> CAN::type2parameters;
int* CAN::alias2masterID_ = nullptr;
unsigned short* CAN::alias2status = nullptr;
DriverParameters** CAN::alias2parameters = nullptr;
int CAN::orderSlaveID2alias[8][16] = {};

CAN::CAN(int order, char const* device)
    : order(order),
      rxSwap(nullptr),
      txSwap(nullptr) {
    (void)device;
}

int CAN::config(){
    // CAN support disabled; treat as no-op success.
    return 0;
}

int CAN::run(std::vector<CAN>&){
    // CAN support disabled; treat as no-op success.
    return 0;
}

CAN::~CAN() = default;
}
