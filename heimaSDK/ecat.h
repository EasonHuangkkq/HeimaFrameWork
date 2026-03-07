/* 
    * ecat.h    
 */

#pragma once

#include "ptr_que.h"
#include "common.h"
#include <map>

namespace DriverSDK{
extern bool ecatMatchByIndex;
class ECAT{
public:
    bool dc, sdoRequestable, regRequestable;
    int order, fd, effectorAlias, sensorAlias, * domainSizes;
    std::map<int, std::string> alias2type;
    long period;
    std::map<int, int> alias2slave, alias2domain;
    std::vector<int> domainDivision;
    ec_domain_t** domains;
    unsigned char** domainPtrs;
    SwapList** rxPDOSwaps, ** txPDOSwaps;
    PtrQue<SDOMsg> sdoRequestQueue, sdoResponseQueue, regRequestQueue, regResponseQueue;
    ec_master_t* master;
    pthread_t pth;
    ECAT(int const order);
    int init();
    int readAlias(unsigned short const slave, std::string const& category, unsigned short const index, unsigned char const subindex, unsigned char const bitLength);
    int requestState(unsigned short const slave, char const* stateString);
    int check();
    int config();
    static void* rxtx(void* arg);
    int run();
    void clean();
    ~ECAT();
};
}
