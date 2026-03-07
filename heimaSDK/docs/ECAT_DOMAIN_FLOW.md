# EtherCAT Domain Flow (Detailed, multi-domain, config.xml example)

This document expands the full end-to-end logic for EtherCAT domains in this SDK.
It uses the current `config.xml` with two domains and two MT_Device slaves.

All paths below are for master order=0, but the logic is identical for each master.

## 0) Example config.xml (two domains)

```
<Domains>
  <Domain master="0" order="0" division="1"/>
  <Domain master="0" order="1" division="1"/>
</Domains>

<Slaves>
  <Slave master="0" domain="0" alias="1" type="MT_Device">1</Slave>
  <Slave master="0" domain="1" alias="2" type="MT_Device">1</Slave>
</Slaves>
```

Meaning:
- There are 2 domains for master 0: domain 0 and domain 1.
- alias 1 belongs to domain 0; alias 2 belongs to domain 1.
- division = 1 for both domains (process every cycle).

## 0.1) Call chain + source map (file:line)

1) `DriverSDK::init()` -> `impClass::init()` creates ConfigXML (`heimaSDK/heima_driver_sdk.cpp:667`, `heimaSDK/heima_driver_sdk.cpp:157`).
2) XML -> `ecatAlias2domain` + `ecatDomainDivision` (`heimaSDK/config_xml.cpp:299`, `heimaSDK/config_xml.cpp:78`) stored at `heimaSDK/heima_driver_sdk.cpp:261-264`.
3) `ECAT` constructor per master pulls the maps and period/DC (`heimaSDK/ecat.cpp:43-69`).
4) `ECAT::init()` allocates per-domain arrays (`heimaSDK/ecat.cpp:74-106`).
5) `ECAT::check()` validates alias->domain and matches on-bus slaves (`heimaSDK/ecat.cpp:176-269`).
6) `ECAT::config()` creates domains, registers PDOs, assigns offsets, activates, allocates SwapLists (`heimaSDK/ecat.cpp:272-523`).
7) `ECAT::rxtx()` runs the cyclic send/receive loop (`heimaSDK/ecat.cpp:626-820`).
8) App publishes Rx frames by `advanceNodePtr()` in `impClass::ecatUpdate()` (`heimaSDK/heima_driver_sdk.cpp:485-538`).

Note: line numbers are from the current tree; they will shift if files change.

## 1) Config parsing -> global maps

`ConfigXML` parses:
- `alias2domain` from `<Slave ... domain="X" ...>` entries
- `domainDivision` from `<Domain ... division="Y"/>`

Result for the example:
- `alias2domain`: {1 -> 0, 2 -> 1}
- `domainDivision`: [1, 1]

These are stored in global vectors:
- `ecatAlias2domain`
- `ecatDomainDivision`

## 1.1) How domainDivision is built (code detail)

`ConfigXML::domainDivision("ECAT")` (`heimaSDK/config_xml.cpp:78-92`):
1) Walks each `<Domain master="M" order="D" division="K"/>`.
2) Ensures `ret.size() > M` and `ret[M].size() > D` (fills missing with 1).
3) Writes `ret[M][D] = K`.

So `domainDivision[master][order] = division`.

## 1.2) How alias2domain is built (code detail)

`ConfigXML::alias2attribute("ECAT", "domain")` (`heimaSDK/config_xml.cpp:299-319`):
1) Walks each enabled `<Slave ...>` (text == 1).
2) Uses `master` attribute to choose `ret[master]`.
3) Inserts `alias -> domain`.

So `alias2domain[master][alias] = domain`.

## 2) ECAT object construction (per master)

`ECAT::ECAT(order)`:
- Copies the per-master maps:
  - `alias2domain = ecatAlias2domain[order]`
  - `domainDivision = ecatDomainDivision[order]`
- Calls `init()` until it succeeds

Source: `heimaSDK/ecat.cpp:43-69`.

`ECAT::init()` allocates arrays sized by `domainDivision.size()`:

```
domains      = new ec_domain_t*[domainCount];
domainPtrs   = new unsigned char*[domainCount];
domainSizes  = new int[domainCount];
rxPDOSwaps   = new SwapList*[domainCount];
txPDOSwaps   = new SwapList*[domainCount];
```

Source: `heimaSDK/ecat.cpp:74-103`.

Why double pointers:
- Each array element is already a pointer:
  - `domains[i]` is an `ec_domain_t*`
  - `domainPtrs[i]` is an `unsigned char*` (process image start)
  - `rxPDOSwaps[i]` and `txPDOSwaps[i]` are `SwapList*`

So the container is "array of pointers" -> `**`.

Example with two domains:
```
domains      -> [0] ec_domain_t*  (domain 0 handle)
               [1] ec_domain_t*  (domain 1 handle)
domainPtrs   -> [0] unsigned char* (domain 0 process image)
               [1] unsigned char* (domain 1 process image)
domainSizes  -> [0] int (domain 0 byte size)
               [1] int (domain 1 byte size)
rxPDOSwaps   -> [0] SwapList* (3 buffers x domainSize[0])
               [1] SwapList* (3 buffers x domainSize[1])
txPDOSwaps   -> [0] SwapList* (3 buffers x domainSize[0])
               [1] SwapList* (3 buffers x domainSize[1])
```

## 3) Domain validity check

`ECAT::check()` verifies that every alias maps to a valid domain index:

- If `alias2domain[alias] >= domainDivision.size()` -> config error.

This catches XML mistakes before PDO registration.

Source: `heimaSDK/ecat.cpp:176-188`.

## 4) ECAT::config() (domain creation + PDO registration)

Source: `heimaSDK/ecat.cpp:272-486`.

### 4.1 Create each domain

```
for i in [0 .. domainCount-1]:
  domains[i] = ecrt_master_create_domain(master);
```

Now you have a real EtherCAT domain handle for each index.

### 4.2 Register each slave into its domain

For each slave on the bus:
- Lookup its domain: `domain = alias2domain[alias]`
- Build PDO entry lists from XML
- Register entries into `domains[domain]`

```
rxPDOOffset = ecrt_slave_config_reg_pdo_entry(..., domains[domain], &bitPosition);
txPDOOffset = ecrt_slave_config_reg_pdo_entry(..., domains[domain], &bitPosition);
```

These offsets are byte offsets inside that domain's process image.
The process image layout follows the registration order: all Rx entries for a slave,
then all Tx entries, then the next slave in the same domain.

### 4.3 Bind each driver to its domain + offsets

For a driver with alias X:
```
drivers[X-1].init(..., domain, rxPDOOffset, txPDOOffset, ...);
```

So each driver stores:
- which master and domain it belongs to
- where its Rx/Tx data starts inside that domain

### 4.4 DC timing (optional)

If DC is enabled, the cycle time is scaled by `domainDivision[domain]`:
```
ecrt_slave_config_dc(..., domainDivision[domain] * period, ...);
```

## 5) Activation -> process images -> SwapLists

After `ecrt_master_activate(master)`:

```
domainPtrs[i]  = ecrt_domain_data(domains[i]);
domainSizes[i] = ecrt_domain_size(domains[i]);
rxPDOSwaps[i]  = new SwapList(domainSizes[i]);
txPDOSwaps[i]  = new SwapList(domainSizes[i]);
```

Source: `heimaSDK/ecat.cpp:488-511`, `heimaSDK/common.cpp:89-116`.

So each domain gets:
- A process image pointer
- A process image size (bytes)
- One Rx 3-buffer (SwapList)
- One Tx 3-buffer (SwapList)

## 6) SwapList (3-buffer) mechanics

Each `SwapList` has 3 nodes:
- `nodePtr` points to the "current" node
- `previous` and `next` form a ring

Operations:
- `copyTo(domainPtr, size)` -> copies `previous` node to domain memory
- `copyFrom(domainPtr, size)` -> copies domain memory into `next`, then advances nodePtr
- `advanceNodePtr()` -> manually advance current node (publish new data)

Source: `heimaSDK/common.cpp:89-116`.

Layout (ring):
```
nodePtr -> [A] current
          [B] next
          [C] previous
          (then back to A)
```

Important: the copy uses the whole `domainSize` for that domain. There is no per-motor buffer.

## 6.1) Rx path (App -> EtherCAT)

1) App writes control data into the current Rx buffer via `drivers[i].rx->...`.
2) App calls `advanceNodePtr()` to publish this frame so it becomes `previous`.
   - Example: `impClass::ecatUpdate()` advances each domain swap (`heimaSDK/heima_driver_sdk.cpp:485-538`).
3) rxtx thread sends: `copyTo(domainPtr, domainSize)` uses **previous** (`heimaSDK/common.cpp:108-110`) then `ecrt_domain_queue()` (`heimaSDK/ecat.cpp:758-763`).

So the EtherCAT thread never reads the same buffer the app is writing.

## 6.2) Tx path (EtherCAT -> App)

1) rxtx thread receives and processes a domain (`heimaSDK/ecat.cpp:792-820`).
2) `copyFrom(domainPtr, domainSize)` copies into **next** and advances `nodePtr` (`heimaSDK/common.cpp:112-116`).
3) App reads `drivers[i].tx->...` from the current node.

## 6.3) Atomic boundary (what is / is not atomic)

- `nodePtr` is atomic (pointer switch only).
- `memcpy` is not atomic; the code relies on the triple-buffer ring to avoid read/write overlap.

## 6.4) Swap buffers vs domainPtr layout

- `SwapList` buffers are just raw byte arrays (`calloc`, `heimaSDK/common.cpp:79-116`).
- `domainPtr` is also a raw byte array from SOEM (`ecrt_domain_data`).
- `copyTo` / `copyFrom` uses **domainSize** bytes, so the layouts must match.
- The offsets returned by `ecrt_slave_config_reg_pdo_entry` make the layouts align for each device.

## 7) Binding drivers to SwapLists

After domain activation, each driver is bound to its domain swap:

```
drivers[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i]);
```

This sets:
- `drivers[j].rx.swap = rxPDOSwaps[i]`
- `drivers[j].tx.swap = txPDOSwaps[i]`

So driver Rx/Tx access always goes to the correct domain buffers.

## 8) Runtime rxtx thread (per master)

The rxtx loop repeats at `period`:

### 8.1 Send (Rx)

```
if (rxPDOSwaps[i] != nullptr && count % domainDivision[i] == 0) {
  rxPDOSwaps[i]->copyTo(domainPtrs[i], domainSizes[i]);
  ecrt_domain_queue(domains[i]);
}
ecrt_master_send(master);
```

Meaning:
- For each domain i, if its division allows this cycle:
  - Copy the previous Rx buffer into that domain process image
  - Queue the domain for transmission

Source: `heimaSDK/ecat.cpp:758-766`.

### 8.2 Receive (Tx)

```
ecrt_master_receive(master);
if (txPDOSwaps[i] != nullptr && count % domainDivision[i] == 0) {
  ecrt_domain_process(domains[i]);
  ecrt_domain_state(domains[i], &domainStates[i]);
  if (domainStates[i].wc_state == EC_WC_COMPLETE) {
    txPDOSwaps[i]->copyFrom(domainPtrs[i], domainSizes[i]);
  }
}
```

Meaning:
- For each domain i, if its division allows this cycle:
  - Process the domain
  - If working counter is complete:
    - Copy domain process image into Tx swap and advance the node

Source: `heimaSDK/ecat.cpp:792-820`.

### 8.3 domainDivision effect

- `division = 1` -> every cycle
- `division = 2` -> every 2 cycles (downsampled domain updates)

Example timeline (count starts at 0):
```
count:     0 1 2 3 4 5
domain 0:  S . S . S .   (division=2, send/recv on even counts)
domain 1:  S S S S S S   (division=1, every cycle)
```

`S` means both `copyTo` (send) and `copyFrom` (receive) run for that domain.

## 9) Concrete example (two domains, MT_Device)

From your logs:
```
master 0, domain 0, domainSize 32
master 0, domain 1, domainSize 32
rxPDOOffset 0, txPDOOffset 16
```

MT_Device layout is 16 bytes Rx + 16 bytes Tx.

So per domain:
```
offset 0..15  : DriverRxData (ControlWord, TargetPos, TargetVel, ...)
offset 16..31 : DriverTxData (StatusWord, ActualPos, ActualVel, ...)
```

Mapping:
- Domain 0 process image contains only alias 1
- Domain 1 process image contains only alias 2

When rxtx runs, it copies the **entire** 32-byte domain image each cycle:
- `copyTo(domainPtr, 32)` for Rx
- `copyFrom(domainPtr, 32)` for Tx

Buffers:
```
rxPDOSwaps[0]/txPDOSwaps[0] -> alias 1
rxPDOSwaps[1]/txPDOSwaps[1] -> alias 2
```

## 9.1) If multiple motors share one domain (example)

Suppose domain 0 has 3 MT_Device motors:
- Each motor still has its own `rxPDOOffset` / `txPDOOffset` inside domain 0.
- `domainSize` becomes the sum of all PDO bytes in domain 0 (e.g. 3 * 32 = 96 bytes).
- There is still **one** `rxPDOSwaps[0]` and **one** `txPDOSwaps[0]` (each is 3 buffers of 96 bytes).
- All three motors read/write inside the same swap buffers, each at its offset.

So the triple buffer is **per domain**, not per motor.

## 10) Driver access path (why offsets matter)

The driver accesses its data via `DataWrapper`:
- `rx` and `tx` wrappers store an `offset`
- `rx->field` returns `swap->nodePtr->memPtr + offset` cast as `DriverRxData*`
  - Implementation: `DataWrapper::operator->()` (`heimaSDK/common.h:231-235`)
- `previous()` and `next()` use the ring nodes (`heimaSDK/common.h:225-241`)

So the driver does not "own" memory; it points into the domain's swap buffer.
The `offset` comes from `ecrt_slave_config_reg_pdo_entry` when each slave is registered
(`heimaSDK/ecat.cpp:395-424`).

## 11) Summary

- `domainDivision.size()` controls how many domains exist.
- Arrays are "array of pointers" because each domain has its own pointer.
- Each domain has its own process image and its own Rx/Tx SwapList pair.
- PDO registration places each slave's entries into its assigned domain.
- Runtime loop copies domain-sized buffers to/from EtherCAT per domain and per division.
