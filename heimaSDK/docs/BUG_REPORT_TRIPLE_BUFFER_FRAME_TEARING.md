# Bug Report: Triple-Buffer Frame Tearing in DriverSDK EtherCAT Data Path

| Field | Value |
|---|---|
| **Component** | `heimaSDK / common.{h,cpp}, ecat.cpp, heima_driver_sdk.cpp` |
| **Priority** | **P0** — Data integrity violation in safety-critical real-time control loop |
| **Severity** | **S1** — Affects correctness of every motor control cycle; silent corruption |
| **Type** | Concurrency / Data Race / Design Defect |
| **Status** | Open |
| **Reporter** | DriverSDK Audit |
| **Date** | 2026-04-06 |
| **Reproducibility** | Deterministic under specific thread-scheduling phase alignment; probabilistic in practice |
| **Affected Versions** | All current versions (structural defect since initial implementation) |

---

## 1. Executive Summary

The lock-free triple buffer (`SwapList`) used for EtherCAT PDO exchange between the real-time (RT) thread and the application thread has a **structural design defect** that causes **intra-frame data tearing**. When the application thread reads motor feedback via `getMotorActual()` or makes control decisions in `setMotorTarget()`, individual field accesses (`ActualPosition`, `ActualVelocity`, `ActualTorque`, etc.) each perform an independent `atomic<SwapNode*>::load()`. If the RT thread publishes a new EtherCAT frame between any two field reads, those fields silently come from **different physical bus cycles** (frame N vs frame N+1).

This is not a theoretical concern. At 1 kHz control frequency with both threads running concurrently, the probability of hitting this race window is **non-negligible on every single control cycle**. The result is a **stitched sensor reading** that never existed in reality — a phantom state fed directly into the control loop.

---

## 2. Architecture Context

### 2.1 Thread Model

```
┌──────────────────────────────────────────────────────────────────────┐
│                         APPLICATION THREAD                            │
│  Frequency: ~1 kHz (driven by loong_base task_locomotion)            │
│  Priority:  Normal (SCHED_OTHER) or elevated (SCHED_FIFO)            │
│  CPU:       Not pinned (may be preempted by OS scheduler)            │
│                                                                      │
│  Calls:                                                              │
│    getMotorActual(data)  → reads  txPDOSwap (feedback: slave→app)    │
│    setMotorTarget(data)  → reads  txPDOSwap (for state machine)      │
│                          → writes rxPDOSwap (command: app→slave)     │
│                          → calls  ecatUpdate() to publish            │
└──────────────────────────┬──────────────────────┬────────────────────┘
                           │ rxPDOSwap            │ txPDOSwap
                           │ (command downlink)   │ (feedback uplink)
                           ▼                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         ECAT RT THREAD                                │
│  Frequency: 1 kHz (hardware-locked via clock_nanosleep)              │
│  Priority:  Real-time (SCHED_FIFO, highest)                          │
│  CPU:       Pinned (pthread_setaffinity_np)                          │
│                                                                      │
│  Loop (ecat.cpp ECAT::rxtx):                                        │
│    1. sleep until next period                                        │
│    2. ecrt_master_receive()          — DMA from NIC                  │
│    3. ecrt_domain_process()          — decode into domainPtrs[]      │
│    4. txPDOSwap->copyFrom(domainPtr) — publish feedback to app       │
│    5. rxPDOSwap->copyTo(domainPtr)   — consume command from app      │
│    6. ecrt_domain_queue() + ecrt_master_send()                       │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.2 Triple Buffer Design Intent

The `SwapList` is a 3-node circular ring (`A ↔ B ↔ C ↔ A`) with one `atomic<SwapNode*> nodePtr`. The **intent** is a single-producer/single-consumer (SPSC) lock-free buffer where:

- **Producer** publishes by writing to `nodePtr->next`, then advancing `nodePtr` to `next`.
- **Consumer** reads from `nodePtr` (the most recently published node).
- The **third node** serves as a free buffer to absorb timing differences.

### 2.3 Data Geometry

Each SwapList holds one full EtherCAT domain buffer. Multiple motors share the same domain. Motor data is accessed via `DataWrapper<T>`, which overlays a typed view at a fixed byte offset within the domain buffer:

```
SwapNode::memPtr ──────────────────────────────────────────────────
  │ offset 0    │ Motor 1 TxPDO (22 bytes: StatusWord, ActualPos, ActualVel, ActualTor, ...) │
  │ offset 22   │ Motor 2 TxPDO (22 bytes) │
  │ offset 44   │ Motor 3 TxPDO (22 bytes) │
  │ ...         │                           │
  ──────────────────────────────────────────────────────────────────
```

---

## 3. Root Cause Analysis

### 3.1 The Core Defect: Per-Field Pointer Reload

`DataWrapper::operator->()` in `common.h:230-235`:

```cpp
Data* operator->(){
    if(swap != nullptr){
        return (Data*)(swap->nodePtr.load()->memPtr + offset);
        //              ^^^^^^^^^^^^^^^^
        //              FRESH atomic load on EVERY dereference
    }
    return data;
}
```

Every C++ expression of the form `drivers[i].tx->SomeField` compiles to:

1. Load `swap->nodePtr` (atomic load → returns `SwapNode*`)
2. Dereference `->memPtr`
3. Add `offset`
4. Cast to `Data*`
5. Read `SomeField` from the resulting pointer

**Crucially, step 1 is repeated independently for each field access.** Two consecutive expressions:

```cpp
drivers[i].tx->ActualPosition    // atomic load #1 → may return node A
drivers[i].tx->ActualVelocity    // atomic load #2 → may return node B
```

...can observe **different** published nodes if the RT thread executes `copyFrom()` (which advances `nodePtr`) between the two loads.

### 3.2 The RT Publisher: `copyFrom()` in `common.cpp:112-116`

```cpp
void SwapList::copyFrom(unsigned char const* domainPtr, int const domainSize){
    SwapNode* node = nodePtr.load();           // capture current
    memcpy(node->next->memPtr, domainPtr, domainSize);  // write to next
    nodePtr.store(node->next);                 // publish: advance nodePtr
}
```

This is called from `ECAT::rxtx()` at `ecat.cpp:836`:

```cpp
ecat->txPDOSwaps[i]->copyFrom(ecat->domainPtrs[i], ecat->domainSizes[i]);
```

The `nodePtr.store(node->next)` is the **publish point**. Any app-thread `nodePtr.load()` occurring after this store will observe the new frame. Any load before will observe the old frame. When field accesses straddle this boundary, the result is a **torn read**.

### 3.3 Why This Is Not Caught by Testing

- The bug produces **plausible but slightly wrong values**. A position from frame N and velocity from frame N+1 differ by at most one 1ms-tick of motion — typically microradians or sub-mm/s. This is within noise margins for casual observation.
- The bug is **phase-dependent**: it only manifests when the app thread's read window overlaps the RT thread's publish point. With both at 1 kHz, this depends on scheduling jitter.
- There are **no checksums, sequence numbers, or coherence assertions** in the data path.
- Hardware smoke tests run open-loop or very simple closed-loop, where 1-sample inconsistency is absorbed by the controller's robustness margins.

---

## 4. Detailed Bug Catalog

### BUG-1: Intra-Motor Field Tearing (Feedback Path)

| Attribute | Detail |
|---|---|
| **Location** | `heima_driver_sdk.cpp:986-1014` (`getMotorActual()`) |
| **Severity** | **P0/S1 — Critical** |
| **Type** | Data race / snapshot inconsistency |
| **Deterministic** | Yes, given specific thread interleaving |

**Affected code:**

```cpp
// heima_driver_sdk.cpp:997-1005
// Each line does an independent nodePtr.load() via operator->()
data[i].pos        = 2.0 * Pi * ... * (drivers[i].tx->ActualPosition - ...) / ...;  // load #1
data[i].vel        = 2.0 * Pi * ... *  drivers[i].tx->ActualVelocity / ...;          // load #2
data[i].tor        = ... * drivers[i].tx->ActualTorque / ...;                        // load #3
data[i].temp       = drivers[i].tx->MotorTemperature;                                // load #4
data[i].driveTemp  = drivers[i].tx->DriveTemperature;                                // load #5
data[i].voltage    = drivers[i].tx->Voltage;                                         // load #6
data[i].statusWord = drivers[i].tx->StatusWord;                                      // load #7
data[i].errorCode  = drivers[i].tx->ErrorCode;                                       // load #8
```

**Impact:**

For one motor in one control cycle, the returned `motorActualStruct` may contain:
- `pos` from EtherCAT frame N
- `vel` from EtherCAT frame N+1
- `tor` from EtherCAT frame N+1
- `statusWord` from EtherCAT frame N+1

This phantom state was **never physically measured** — it is a chimera of two consecutive bus cycles.

**Downstream consequences:**

| Consumer | Impact |
|---|---|
| Velocity estimation (numerical differentiation of position) | Discontinuity / spike when position and velocity come from different frames |
| Torque feedforward | Mismatch between inertial state (pos/vel) and torque reading |
| Fault detection | `statusWord` from frame N+1 may show fault while `pos`/`vel` from frame N appear normal → delayed or missed fault response |
| State estimation (EKF/UKF) | Inconsistent observation vector → filter divergence or innovation spikes |
| Data logging / replay | Logged data contains phantom states that cannot be reproduced |

---

### BUG-2: Inter-Motor Frame Skew (Feedback Path)

| Attribute | Detail |
|---|---|
| **Location** | `heima_driver_sdk.cpp:986-1014` (`getMotorActual()` loop over `dofAll`) |
| **Severity** | **P0/S1 — Critical** |
| **Type** | Cross-axis snapshot inconsistency |
| **Deterministic** | Yes, given specific thread interleaving |

**Mechanism:**

`getMotorActual()` iterates over motors `i = 0 .. dofAll-1` sequentially. All motors in the same domain share the same `txPDOSwap`. If the RT thread publishes between processing motor `i=5` and motor `i=6`:

```
Timeline:
  App:  read motor 0 fields  → frame N  ✓
  App:  read motor 1 fields  → frame N  ✓
  ...
  App:  read motor 5 fields  → frame N  ✓
  RT:   copyFrom() → nodePtr advances from A to B (frame N → N+1)
  App:  read motor 6 fields  → frame N+1  ✗ INCONSISTENT
  ...
  App:  read motor 12 fields → frame N+1  ✗ INCONSISTENT
```

**Impact:**

| Consumer | Impact |
|---|---|
| Forward Kinematics (FK) | Joint angles from different time instants → impossible body configuration |
| Inverse Dynamics | Inconsistent joint states → incorrect computed torques |
| Gait state machine | Left leg from frame N, right leg from frame N+1 → phase estimation error |
| Whole-body balance controller | CoM estimation based on phantom configuration → balance disturbance |
| Safety envelope checks | Joint limit / workspace limit checks on non-physical configuration → false positive or false negative |

---

### BUG-3: State Machine Decisions Based on Torn Feedback

| Attribute | Detail |
|---|---|
| **Location** | `heima_driver_sdk.cpp:858-896` (`setMotorTarget()` Step 1: CiA-402 state machine) |
| **Severity** | **P1/S2 — High** |
| **Type** | Control logic error from inconsistent input |

**Affected code:**

```cpp
// heima_driver_sdk.cpp:862-871
switch(drivers[i].tx->StatusWord & 0x007f){    // nodePtr.load() → frame N or N+1
case 0x0033:
    drivers[i].rx->Mode = operatingMode[i];
    drivers[i].rx->ControlWord = 0x0f;
    drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
    //                               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //                               SECOND nodePtr.load() → possibly different frame!
    break;
}
```

**Impact:**

During the `NOT_READY → SWITCH_ON_DISABLED → READY_TO_SWITCH_ON → SWITCHED_ON → OPERATION_ENABLED` transition:

- `StatusWord` may indicate state `0x0033` (Switched On) from frame N
- `ActualPosition` captured for initial position may come from frame N+1
- The position difference between frame N and N+1 at 1 kHz is typically small, but during fast motions or high-gear-ratio joints, this can be several encoder counts
- This mismatch can cause a **position jump** at the moment the drive enters Operation Enabled

---

### BUG-4: Unbounded Reader Lifetime — RT Can Overwrite Active Read Buffer

| Attribute | Detail |
|---|---|
| **Location** | `common.h:224-235` (`DataWrapper::operator->()`, `previous()`, `next()`) |
| **Severity** | **P1/S2 — High (latent)** |
| **Type** | Use-after-publish / data race (C++ UB) |

**Mechanism:**

The triple buffer has 3 nodes. After the RT thread publishes 3 times, the node that the app thread was reading becomes the RT thread's next write target:

```
Initial state:  nodePtr → A (app reading A)
RT publish #1:  write to B, nodePtr → B
RT publish #2:  write to C, nodePtr → C
RT publish #3:  write to A, nodePtr → A  ← RT is now memcpy-ing INTO node A
                                           while App still holds a pointer to A!
```

This requires the app thread to be stalled for ≥ 2ms (2 RT cycles at 1kHz) while holding a raw pointer into a swap node. Scenarios where this occurs:

- App thread is `SCHED_OTHER` and gets preempted by a higher-priority task
- Page fault during field access
- CPU frequency scaling event
- System under memory pressure (swapping)
- GDB/strace attached for debugging

Under the C++ memory model, concurrent `memcpy` write (RT) and field read (App) on the same buffer is **undefined behavior**, not merely stale data.

**Note:** This does NOT require a code bug or programmer error to trigger. It is an inherent limitation of the current design when the reader is not real-time.

---

### BUG-5: ecatUpdate() Converter Path — Quadruple Advance on 3-Node Ring

| Attribute | Detail |
|---|---|
| **Location** | `heima_driver_sdk.cpp:500-520` (`ecatUpdate()` converter handling) |
| **Severity** | **P2/S3 — Medium** |
| **Type** | Buffer aliasing |

**Affected code:**

```cpp
// heima_driver_sdk.cpp:502-520
// For RS485Emu converters, the SAME rxPDOSwap is advanced 3 times here:
ecats[...].rxPDOSwaps[...domain...]->advanceNodePtr();  // advance #1
// ... write channel data ...
ecats[...].rxPDOSwaps[...domain...]->advanceNodePtr();  // advance #2
// ... write channel data ...
ecats[...].rxPDOSwaps[...domain...]->advanceNodePtr();  // advance #3
// ... write channel data ...

// Then at line 529-539, the SAME swap is advanced AGAIN:
ecats[i].rxPDOSwaps[j]->advanceNodePtr();               // advance #4
```

On a 3-node ring, 3 advances returns to the **same node**. The 4th advance moves to `next`. But during advances 1-3, the "current" node passed through all three buffers, meaning the RT thread's `copyTo` (which reads `nodePtr->previous`) could read a buffer that was just written to in the same function call.

---

### NON-BUG: Memory Ordering (Low Risk, Maintainability Concern)

The current code uses default `memory_order_seq_cst` for all atomic operations, which is **stronger than necessary but correct**. The publication pattern:

```
Producer:  memcpy(target, source, size);     // write payload
           nodePtr.store(target);             // publish (seq_cst ⊇ release)

Consumer:  SwapNode* p = nodePtr.load();      // acquire (seq_cst ⊇ acquire)
           read(p->memPtr);                   // read payload
```

...is a valid release-acquire publication. No fence bugs today. However:

- `seq_cst` on every load/store is unnecessarily expensive on ARM (full barrier vs acquire/release which map to `ldapr`/`stlr`)
- Future refactoring might accidentally weaken the ordering without understanding the implicit contract

**Recommendation:** Make ordering explicit: `store(..., memory_order_release)` and `load(memory_order_acquire)`.

### NON-BUG: `advanceNodePtr()` Load-Store (Not CAS)

```cpp
void SwapList::advanceNodePtr(){
    nodePtr.store(nodePtr.load()->next);  // NOT atomic RMW
}
```

This is **safe under the current single-publisher model**:
- `rxPDOSwap`: only app thread publishes
- `txPDOSwap`: only RT thread publishes (via `copyFrom`)

No competing publishers means no lost-update risk. CAS is unnecessary.

**However:** This invariant is not documented or enforced. Adding a second publisher without redesigning would introduce silent data corruption.

---

## 5. Probability of Occurrence

### Simplified Model

At 1 kHz, the RT thread publishes every 1ms. The "danger window" is the duration of the `nodePtr.store()` instruction — effectively instantaneous. But the **vulnerable window** for the app thread is the entire duration of `getMotorActual()` or `setMotorTarget()`.

```
Let:
  T_rt    = 1 ms    (RT period)
  T_read  = time for App to read all motors in one domain
  N_loads = number of independent nodePtr.load() calls per getMotorActual() invocation
          = dofAll × 8 fields = 13 × 8 = 104 loads (for 13-motor config)

If T_read ≈ 10-50 µs (typical for 13 motors, no preemption):
  P(torn) ≈ T_read / T_rt ≈ 1-5% per call

If App is preempted during read (adds 50-500 µs):
  P(torn) ≈ 5-50% per call
```

At 1 kHz call rate:
- **Without preemption**: ~10-50 torn reads per second
- **With occasional preemption**: ~50-500 torn reads per second

These are **not rare events**. They are a steady background corruption rate.

---

## 6. Reproduction Strategy

### 6.1 Instrumented Detection (Non-Invasive)

Add a **sequence counter** to detect tearing without changing the buffer design:

```cpp
// In ECAT::rxtx(), after copyFrom():
static thread_local uint64_t txCycleCounter = 0;
txCycleCounter++;
// Write counter to a reserved field or a side-channel shared variable

// In getMotorActual(), read the counter before and after:
uint64_t seq_before = txCycleCounter_shared.load(std::memory_order_acquire);
// ... read all motor fields ...
uint64_t seq_after = txCycleCounter_shared.load(std::memory_order_acquire);
if(seq_before != seq_after) {
    torn_read_count++;  // LOG THIS
}
```

### 6.2 Stress Test (Maximizes Race Window)

```cpp
// In a test harness, run getMotorActual() in a tight loop
// while the RT thread is active. Log all readings.
// Compare consecutive pos/vel pairs:
//   If |pos[t] - pos[t-1]| is small but |vel[t]| is large (or vice versa),
//   the sample is likely torn.

// Alternatively, compute: vel_estimated = (pos[t] - pos[t-1]) / dt
// Compare with vel[t]. Large discrepancy = torn read.
```

### 6.3 Synthetic Reproduction (Unit Test, No Hardware)

```cpp
// Create a SwapList with 3 nodes.
// Thread A (simulating RT): in a tight loop, write known patterns to nodes
//   via copyFrom(). Pattern: all bytes = cycle_count & 0xFF.
// Thread B (simulating App): read via operator->(), check that all bytes
//   in one "struct" come from the same cycle.
// Any mismatch = confirmed torn read.
```

---

## 7. Proposed Fix

### 7.1 Approach: Per-Domain Snapshot Before Field Access

The minimal correct fix is to **load `nodePtr` once per domain per API call** and read all fields from that single snapshot. This eliminates all per-field reload races.

### 7.2 Option A: Shadow Copy (Recommended — Simplest, Safest)

Add a snapshot method to `SwapList` and copy the entire domain buffer to a thread-local or stack-allocated shadow:

```cpp
// common.h — add to SwapList:
void snapshotTo(unsigned char* dst, int const size) const {
    SwapNode* node = nodePtr.load(std::memory_order_acquire);
    memcpy(dst, node->memPtr, size);
}
```

In `getMotorActual()`:

```cpp
int getMotorActual(vector<motorActualStruct>& data) {
    // Phase 1: Snapshot all domains
    // Stack-allocate shadow buffers (domain sizes are typically < 1KB)
    for(int e = 0; e < ecats.size(); e++) {
        for(int d = 0; d < ecats[e].domainDivision.size(); d++) {
            if(ecats[e].txPDOSwaps[d] != nullptr) {
                ecats[e].txPDOSwaps[d]->snapshotTo(
                    txShadow[e][d],  // thread-local or stack buffer
                    ecats[e].domainSizes[d]
                );
            }
        }
    }

    // Phase 2: Read all motors from shadow (zero race risk)
    for(int i = 0; i < dofAll; i++) {
        auto* tx = (DriverTxData*)(txShadow[drivers[i].order][drivers[i].domain]
                                   + drivers[i].tx.offset);
        data[i].pos = ... tx->ActualPosition ...;  // from shadow
        data[i].vel = ... tx->ActualVelocity ...;  // from shadow — SAME frame
        data[i].tor = ... tx->ActualTorque   ...;  // from shadow — SAME frame
        // ... all fields from same snapshot
    }
}
```

**Pros:**
- Zero-risk: shadow is owned by the app thread, cannot be overwritten by RT
- Also fixes BUG-4 (unbounded reader lifetime)
- Trivial to implement — no SwapList redesign needed
- `memcpy` of ~300 bytes per domain is negligible at 1 kHz (~0.3 µs)

**Cons:**
- One extra copy per domain per cycle (negligible cost)

### 7.3 Option B: Pinned Pointer (Zero-Copy, Requires Discipline)

Add a method that returns a pinned snapshot pointer, used for the duration of one API call:

```cpp
// common.h — add to SwapList:
SwapNode* acquireReadPtr() const {
    return nodePtr.load(std::memory_order_acquire);
}
```

Modify `DataWrapper` to support snapshot reads:

```cpp
template<typename Data>
class DataWrapper {
    // ... existing members ...

    // NEW: Read from a specific snapshot (not from live nodePtr)
    Data* fromSnapshot(SwapNode* snapshot) {
        return (Data*)(snapshot->memPtr + offset);
    }
};
```

**Pros:**
- True zero-copy
- No `memcpy` overhead

**Cons:**
- Does NOT fix BUG-4 (if app holds pointer too long, RT can overwrite)
- Requires caller discipline to not hold pointer across cycles
- Harder to audit for correctness

### 7.4 Option C: Proper SPSC Triple Buffer (Gold Standard)

Replace `SwapList` with a well-proven SPSC triple buffer design using **three explicit state slots** (WRITING, LATEST, READING) with atomic index exchange:

```cpp
class TripleBuffer {
    alignas(64) unsigned char buffers[3][MAX_DOMAIN_SIZE];
    std::atomic<int> latest{0};    // index of most recently published buffer
    int writing{1};                // producer-private: index being written
    int reading{2};                // consumer-private: index being read

public:
    // Producer (RT thread):
    unsigned char* writeBuffer() { return buffers[writing]; }
    void publish() {
        writing = latest.exchange(writing, std::memory_order_acq_rel);
    }

    // Consumer (App thread):
    unsigned char* readBuffer() {
        reading = latest.exchange(reading, std::memory_order_acq_rel);
        return buffers[reading];
    }
};
```

**Pros:**
- Formally correct SPSC design
- Writer never blocks
- Reader always gets the latest complete frame
- Reader's buffer is **exclusively owned** — cannot be overwritten (fixes BUG-4)
- Well-studied design with known correctness proofs

**Cons:**
- Larger refactor (replace `SwapList` entirely)
- Need to update all `DataWrapper` and `WrapperPair` plumbing

### 7.5 Recommendation

| Timeframe | Action | Fixes |
|---|---|---|
| **Immediate (1-2h)** | **Option A**: Shadow copy in `getMotorActual()` and `setMotorTarget()` Step 1 | BUG-1, BUG-2, BUG-3, BUG-4 |
| **Short-term (4-8h)** | Explicit `memory_order_acquire`/`release` + document single-publisher invariant | Maintainability |
| **Medium-term (1-2d)** | **Option C**: Replace `SwapList` with proper SPSC triple buffer | All bugs + architectural soundness |
| **Optional** | Add per-cycle sequence stamp for cross-domain coherence validation | Multi-domain skew detection |

---

## 8. Scope of Impact

### 8.1 Affected API Surface

| API | Read/Write | Affected by |
|---|---|---|
| `getMotorActual()` | Read txPDOSwap | BUG-1, BUG-2, BUG-4 |
| `setMotorTarget()` Step 1 (CiA-402 SM) | Read txPDOSwap | BUG-1, BUG-3 |
| `setMotorTarget()` Step 2 (write targets) | Write rxPDOSwap | Not directly torn (single-writer) |
| `getDigitActual()` | Read txPDOSwap | BUG-1 (same pattern) |
| `setDigitTarget()` | Write rxPDOSwap | Low risk (single-writer) |
| `getSensor()` | Read txPDOSwap | BUG-1 (same pattern) |
| `getEncoderCount()` | Read txPDOSwap | BUG-1 (same pattern) |
| `getIMU()` | Read RS232 txSwap | Same design, same bug class (separate SwapList for IMU) |

### 8.2 Affected Downstream Systems

| System | File | Impact |
|---|---|---|
| `loong_base-main/main/locomotion/task_locomotion.cpp` | Calls `getMotorActual()` every cycle | Torn joint state → locomotion control error |
| `loong_base-main/main/driver/task_driver.cpp` | Calls `setMotorTarget()` every cycle | Torn feedback → wrong CiA-402 decisions |
| `loong_base-main/main/interface/task_interface.cpp` | Reads and relays motor state | Torn state forwarded to external consumers |
| `loong_sim-main` (MuJoCo sim) | Does NOT use DriverSDK | **Not affected** (direct Nabo call) |
| `heima_simulator` | Uses DriverSDK in real-robot mode | **Affected** |
| `heima_rl_deploy` | Uses DriverSDK | **Affected** |

---

## 9. Test Plan for Fix Verification

### 9.1 Unit Test: Snapshot Consistency

```
GIVEN: A SwapList with 3 nodes, RT thread publishing at 1 kHz
WHEN:  App thread calls snapshotTo() and reads all fields
THEN:  All fields in the snapshot come from the same publish cycle
VERIFY: Embed a monotonic sequence number as the first 8 bytes of each frame.
        After snapshotTo(), assert all 8-byte headers across the snapshot are identical.
```

### 9.2 Integration Test: Cross-Motor Consistency

```
GIVEN: 13-motor configuration, RT thread active
WHEN:  App thread calls getMotorActual() 100,000 times
THEN:  For each call, verify:
       - vel_estimated = (pos[t] - pos[t-1]) / dt matches vel[t] within tolerance
       - All motors in same domain have same sequence number
METRIC: 0 torn reads out of 100,000 calls (was: ~1,000-5,000 before fix)
```

### 9.3 Stress Test: Preemption Resilience

```
GIVEN: App thread running as SCHED_OTHER with competing CPU load
WHEN:  getMotorActual() called at 1 kHz for 1 hour
THEN:  No torn reads detected (sequence number check)
       No ASAN/TSAN violations (data race check)
```

### 9.4 Performance Test: Shadow Copy Overhead

```
GIVEN: 13-motor config, domain size ~300 bytes
WHEN:  Measure wall-clock time of getMotorActual() before and after fix
THEN:  Added latency from memcpy ≤ 1 µs (budget: 1000 µs per cycle)
```

---

## 10. Related Considerations

### 10.1 Cross-Domain Coherence

The proposed fix ensures intra-domain consistency. If a robot has motors split across **multiple EtherCAT masters/domains**, the snapshots from different domains may still come from different bus cycles. This is acceptable if domains are on separate physical buses (inherently asynchronous), but should be documented.

For applications requiring whole-robot coherence across domains, add a **global epoch counter** incremented after all domains are processed, and verify epoch consistency in the app thread.

### 10.2 IMU Path (RS232)

The YeSense IMU uses the same `SwapList` pattern via `RS232::txSwap`. The `getIMU()` function (`heima_driver_sdk.cpp:761-793`) reads RPY, gyroscope, and accelerometer with multiple independent `operator->()` calls, making it susceptible to the same intra-reading tearing.

### 10.3 DataWrapper API Safety

Consider deprecating `DataWrapper::operator->()` for cross-thread use entirely and replacing it with an explicit snapshot API that makes the single-load-per-read semantics visible in the type system:

```cpp
// Instead of:  drivers[i].tx->ActualPosition  (implicit, unsafe)
// Use:         auto snap = drivers[i].tx.snapshot(txSnapshots[domain]);
//              snap->ActualPosition            (explicit, safe)
```

This makes the correct pattern easy and the incorrect pattern impossible.

---

## 11. References

- `heimaSDK/common.h:205-245` — `DataWrapper` template (root cause: `operator->()`)
- `heimaSDK/common.cpp:89-116` — `SwapList` implementation (`advanceNodePtr`, `copyTo`, `copyFrom`)
- `heimaSDK/ecat.cpp:661-910` — `ECAT::rxtx()` RT loop (publisher)
- `heimaSDK/heima_driver_sdk.cpp:847-978` — `setMotorTarget()` (consumer + publisher)
- `heimaSDK/heima_driver_sdk.cpp:981-1018` — `getMotorActual()` (consumer)
- `heimaSDK/heima_driver_sdk.cpp:486-540` — `ecatUpdate()` (rxPDOSwap publisher)
- `heimaSDK/docs/DATAFLOW.md` — Architecture reference
- `heimaSDK/docs/ECAT_TRIPLE_BUFFER_FLOW.md` — Triple buffer design reference
