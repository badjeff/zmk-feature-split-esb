# ZMK ESB Split Transport

This is a [ZMK](https://zmk.dev) *Split Transport* module adding support for [Enhanced ShockBurst (ESB)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) protocol on Nordic nRF5 Series device.

This work is based on [zmk,wired-split](https://github.com/zmkfirmware/zmk/tree/main/app/src/split/wired), [nRF Connect SDK > ESB Examples](https://docs.nordicsemi.com/bundle/ncs-2.6.4/page/nrf/samples/esb.html) and [ncs-esb-ble-mpsl-demo](https://github.com/too1/ncs-esb-ble-mpsl-demo/).


## What it does

This module uses [nRF Connect SDK (NCS)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html) ESB implementation as communication protocol between ZMK central and peripherals, instead of Zephyr BLE stack. The protocol implementation is supporting two-way data packet communication, packet buffering, packet acknowledgment, and automatic retransmission, etc. All devices could be communicated with predefined semantic address. 

This module also uses [Multi-Protocol Service Layer (MPSL)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/multiprotocol/index.html) library which provides services for multiprotocol applications that allows the nRF5 radio driver to negotiate for transmission timeslots. 

> UPDATE FOR ZMK 0.4: I'm too stupid to make BLE security libraries in NCS 3.1 be compiled on Zephye 4.1.

~~As result, ZMK central allows to pair BLE host as conventional HID input device (keyboard & mouse) and act as an ESB transceiver simultaneously.~~

And all ZMK peripherals talk to ZMK central over ESB only with reduced packet overhead.

~~However, the MCU embedded radio controller in nRF52840 (which i used to develop on) doesn't have enough resource to establish ESB connection and perform BLE advertising & scanning between central and peripheral at the same time.~~

~~In short, central doesn't have timeslots to scan peripherals, and peripheral doesn't have timeslots to advertising itself to central. But, central has configed as ESB Primarily Receiver (PRX) and it has free timeslots to advertise itself to HID host.~~

### TL;DR;
This module has ~~two~~one topologies.
- USB-only Dongle with ONLY ESB is enabling.
   - Dongle connects to HID host via USB.
   - Peripherals connects to Dongle via ESB with same ESB arbitrary address.
   - Min latency is 1ms.
   - Power consumption on TX is a bit less than BLE in long term, it does not keep connected to RX.
   - Sample *zmk-config* for a [Corne 36 keys](https://github.com/foostan/crkbd) with couple [pointabella](https://github.com/badjeff/pointabella) variants and [moudabella](https://github.com/badjeff/moudabella) could be find at [here](https://github.com/badjeff/zmk-config/tree/main/config/boards/shields/donki36).
- ~~Wireless Split Central or Dongle, with BOTH BLE and ESB is enabling.~~
   - ~~Split Central pairs to HID host via BLE.~~
   - ~~Split Peripherals connects to Split Central via ESB.~~
   - ~~Min latency is 7.5ms + 1ms.~~
   - ~~Power consumption is about 7.5mA @ 4.0v on central, v.s. 0.65mA with only BLE enabling.~~
   - ~~Split Central is limited be pairing to **single** BLE host on nRF52840.
     *(NOTE: There is not enough radio resource to perform BLE advertising once it is connected to a paired host. Not tested on nRF53/54)*~~


## Installation

Include this project on your ZMK's west manifest in `config/west.yml`:
```diff
  [...]
  remotes:
+    - name: badjeff
+      url-base: https://github.com/badjeff
+    - name: nrfconnect
+      url-base: https://github.com/nrfconnect
  projects:
+    - name: zmk-feature-split-esb
+      remote: badjeff
+      revision: main
+    - name: sdk-nrf
+      remote: badjeff
+      revision: v3.1-branch+zmk-fixes
+      path: nrf
+    - name: nrfxlib
+      remote: nrfconnect
+      revision: v3.1-branch
+      repo-path: sdk-nrfxlib
+      path: nrfxlib
  [...]
```

> [!NOTE]
> Since ZMK 0.4 uses Zephyr 4.1, a patched version of NCS with couple default values is needed to pass CMake config validation. You could see what were patched in my NCS fork from [here](https://github.com/badjeff/sdk-nrf/commits/v3.1-branch%2Bzmk-fixes/).

Update `{shield}.conf` to enable ESB Split Transport.
```conf
# disable BLE on peripheral
# (DEPRECATED) NOTE: keep default (=y) if want to pairing BLE host on split central, or wireless dongle
CONFIG_ZMK_BLE=n

# disable default split transport on central and peripheral
CONFIG_ZMK_SPLIT_BLE=n
CONFIG_ZMK_SPLIT_WIRED=n

# enable split esb transport
CONFIG_ZMK_SPLIT_ESB=y

# assige a source id on peripheral. (no need for central)
# default zero. give an integer (<256) on all peripheral(s)
CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID=1

# enable ESB TX send request packet payload with ACK bit
# ESB protocol has built-in retransmit counter (default one), if RX does not response ACK properly.
# disable this iif you are pursuing extreme low latency, not much different in real-life.
CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK=y

# The delay between each retransmission of unacknowledged packets
# NOTE: radio will chock if too short
CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY=600

# The number of retransmission attempts before transmission fail
# NOTE: applying less retransmit count on pointer device will lead to lossy but sharper move
CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT=3

# The number of Multi-Protocol Service Layer (MPSL) timeslot sessions
# set 1, if CONFIG_ZMK_BLE is disabled on central or peripherals
# set 2, if CONFIG_ZMK_BLE is enabled on central, which needs BLE and ESB simultaneously
CONFIG_MPSL_TIMESLOT_SESSION_COUNT=1

# Number of message queue size to buffer ESB payload for TX in between multi-protocol service 
# timeslots (CONFIG_MPSL_TIMESLOT_SESSION_COUNT)
CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS=16

# qeuue size for both peripheral (EVENT) and central (CMD)
CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS=16
CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS=4

# another IMPORTANT config for ESB
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048
CONFIG_ESB_MAX_PAYLOAD_LENGTH=48

# currently, set ESB_TX_FIFO to 1, to workaround ACK issues before NCS 3.1.0 release
# NOTE: should apply to both TX and RX sides
CONFIG_ESB_TX_FIFO_SIZE=1

# Logging!
# CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL_DBG=y

# for battery reporting on RX
# NOTE: CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS is valid for ESB-only, even its look like a BLE-only config.
# NOTE: central will not advertise to any BLE host if ble_profile_count is zero.
#       with simple math:
#       ZMK_BLE_PROFILE_COUNT = (CONFIG_BT_MAX_PAIRED - CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS)
CONFIG_ZMK_BATTERY_REPORTING=y
CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y
CONFIG_BT_MAX_PAIRED=2
CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS=2

# for battery reporting on TX
CONFIG_ZMK_BATTERY_REPORTING=y

# Put on dongle (central side) could help maxing out HID reports via USB (although it looks like its BLE related)
CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE=1
```

And, add ESB arbitrary address to `{shield}.overlay` of your central and peripherals.
```c
/{
    esb_split {
        compatible = "zmk,esb-split";
        // These are arbitrary default addresses. In end user products
        // different addresses should be used for each set of devices.
        base-addr-0 = <0xE7 0xE7 0xE7 0xE7>;
        base-addr-1 = <0xC2 0xC2 0xC2 0xC2>;
        addr-prefix = <0xE7 0xC2 0xC3 0xC4 0xC5 0xC6 0xC7 0xC8>;
    };
};
```


## Local Development Setup

Following steps will guide to setup a structure with symbolic linked dictionaries like one below.
```
+- ${MY_ZMK_DEV_DIR}
   +- zmk
      +- nrf     -> ../zmk-feature-split-esb/nrf
      +- nrfxlib -> ../zmk-feature-split-esb/nrfxlib
   +- zmk-config
   +- zmk-feature-split-esb
      +- nrf
      +- nrfxlib
```

Clone this module repo and pull nRF Connect SDK repos via west.
```shell
cd ${MY_ZMK_DEV_DIR}
git clone https://github.com/badjeff/zmk-feature-split-esb.git
cd zmk-feature-split-esb
west init -l config/
west update nrf
west update nrfxlib
```

Clone main ZMK repo.
```shell
cd ${MY_ZMK_DEV_DIR}
git clone https://github.com/zmkfirmware/zmk.git
cd zmk
export NRF_MODULE_DIRS="../zmk-feature-split-esb/nrf"
export NRFXLIB_MODULE_DIRS="../zmk-feature-split-esb/nrfxlib"
ln -s "${NRF_MODULE_DIRS}" nrf
ln -s "${NRFXLIB_MODULE_DIRS}" nrfxlib
```

Build with *ZMK_EXTRA_MODULES*
```shell
cd ${MY_ZMK_DEV_DIR}
cd zmk
# you'd like to put following lines in an executable
export NRF_MODULE_DIRS="../zmk-feature-split-esb/nrf"
export NRFXLIB_MODULE_DIRS="../zmk-feature-split-esb/nrfxlib"
export ZMK_ESB_MODULE_DIRS="../zmk-feature-split-esb"
export ZMK_MODULE_DIRS="${ZMK_ESB_MODULE_DIRS};${NRF_MODULE_DIRS};${NRFXLIB_MODULE_DIRS}"
export SHIELD="corne_left"
export BOARD="nice_nano@2.0.0"
export ZMK_CONFIG_DIR="../zmk-config"
west build -d "build/${SHIELD}" -b "${BOARD}" -S zmk-usb-logging -- \
  -DZMK_EXTRA_MODULES="${ZMK_MODULE_DIRS}" \
  -DSHIELD="${SHIELD}" -DZMK_CONFIG="${ZMK_CONFIG_DIR}"
```


## License

- See `SPDX-License-Identifier` in each file heading
  * [LicenseRef-Nordic-5-Clause](https://github.com/nrfconnect/sdk-nrf/blob/main/LICENSE) licensed from [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
  * [MIT](https://github.com/zmkfirmware/zmk/blob/main/LICENSE) licensed from [ZMK](https://github.com/zmkfirmware/zmk/)
