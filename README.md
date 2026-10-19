# Domyos Bridge — FTMS bike → Cycling Power sensor for Apple Watch

ESP-IDF (C++17, NimBLE) firmware for an ESP32 that connects to a Decathlon
Domyos bike over Bluetooth (Fitness Machine Service 0x1826) and re-publishes
its power and cadence as a standard Cycling Power sensor (0x1818). The Apple
Watch Indoor Cycle workout can then record power and cadence into Health.

**Status: Milestone 3 done.** A 17-minute Indoor Cycle workout on the Watch
recorded the bike's real power and cadence, including a stop, bike sleep, and
reconnect mid-workout. Fake-data mode and DEBUG logging are both **off**;
re-enable either in menuconfig if needed.

## Layout

```
main/
  main.cpp              NVS + NimBLE init, 1 Hz publish task
  ftms_parser.*         PURE  FTMS 0x2AD2 / 0x2ACC bytes -> structs
  ftms_client.*         NimBLE central: scan, connect, discover, subscribe, reconnect
  crank_synth.*         PURE  cadence over time -> CPS crank revs + event time
  power_filter.*        PURE  holds back one-update power spikes from the bike
  shared_state.*        latest bike data, std::mutex-protected (IDF-free)
  cps_server.*          NimBLE peripheral: CPS + DIS GATT table, advertising, notify
  status_led.*          optional LED (C3 Super Mini GPIO8)
  nimble_includes.hpp   NimBLE headers + #undef of its min/max macros
  Kconfig.projbuild     "Domyos Bridge" menu
test/                   host unit tests (plain CMake + tiny_test.hpp)
tools/env.sh            activates ESP-IDF in your shell
```

## Toolchain on this Mac

ESP-IDF **v5.3.1** is already installed at `~/esp/esp-idf` (tools in
`~/.espressif`). Its Python env was built with macOS's Python 3.9, but
`python3` now resolves to Homebrew 3.14, so a plain `export.sh` fails.
`tools/env.sh` handles this. In every new terminal:

```bash
. tools/env.sh
```

**VS Code:** the ESP-IDF extension (v2.3) is installed, and
`.vscode/settings.json` points it at the existing install through
`idf.customExtraVars`. If it still offers to install ESP-IDF, choose to use
an existing setup at `~/esp/esp-idf` rather than downloading a new one.

## Build, flash, monitor

The target is already set to `esp32c3`. For a fresh clone, run
`idf.py set-target esp32c3` first.

```bash
idf.py build
```

```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

- The C3 Super Mini uses the chip's built-in USB, so it shows up as
  `/dev/cu.usbmodemXXXX`. If flashing fails to connect, hold **BOOT**, tap
  **RESET** (or plug in while holding BOOT), release BOOT, and flash again.
  After flashing a board that was in download mode, press RESET once.
- Exit the monitor with `Ctrl+]`.
- Other chips: `idf.py set-target esp32s3` or `idf.py set-target esp32`.
  Both build cleanly; the status LED is off by default on those targets.
- Settings: `idf.py menuconfig` → **Domyos Bridge** (names, fake-data mode,
  stale timeout, LED pin, verbose logging).

## Host unit tests

They need no ESP-IDF, just Xcode's clang and CMake. The tests run with
ASan/UBSan and the firmware's `-fno-exceptions -fno-rtti`.

```bash
cmake -S test -B build-test -G Ninja && cmake --build build-test && ctest --test-dir build-test --output-on-failure
```

`test/tiny_test.hpp` is a ~40-line header-only runner. I chose it over Unity
or GoogleTest so the tests build with nothing downloaded and without
sourcing IDF. Each `TEST("name") { CHECK…(…); }` registers itself.

## Pairing with the Apple Watch (watchOS 10 or later)

1. Flash the board and open the monitor. You should see
   `Advertising as "Domyos Bridge"`. On the C3 Super Mini the LED is solid,
   because fake mode counts as "bike connected".
2. On the Watch, go to **Settings → Bluetooth**. Under **Health Devices**, tap
   **Domyos Bridge**. If the Watch asks to pair, accept; it's Just Works, with
   no PIN.
3. In the monitor: `Central connected …`, possibly `Encryption on … bonded=1`,
   then `Power measurement notifications ON`. The LED switches to a double
   blink.
4. Start **Workout → Indoor Cycle**. Scroll through the workout views. Power
   (100 W) and cadence (80 rpm) should appear. If they don't, add the Power
   and Cadence views under the workout's **⋯ → edit → Workout Views**.
5. End the workout, then check that the power and cadence samples are in the
   Fitness/Health app.

Optional pre-check without the Watch: in **nRF Connect** on the iPhone,
connect to "Domyos Bridge", open Cycling Power, and enable notifications on
0x2A63. You should see `20-00-64-00-RR-RR-TT-TT` once a second: flags 0x0020,
100 W, crank revs, event time. Disconnect nRF Connect afterwards.

If the Watch acts strangely after the GATT table changes, use **Forget
Device** on the Watch and pair again. To also wipe the ESP's bonds, run
`idf.py -p /dev/cu.usbmodem* erase-flash` and then flash again.

## Reading the logs

### Bike side (`ftms` tag)

```
I ftms: Scanning for an FTMS bike (0x1826 or name "Domyos-Bike*")
I ftms: Connecting to bike xx:xx:xx:xx:xx:xx (type 0, rssi -55)
I ftms: Bike connected (handle=0); discovering FTMS
I ftms: Bike MTU=…
I ftms: Fitness Machine Feature: features=0x…  target_settings=0x…
I ftms:   bike claims: [Cadence] [Total Distance] [Resistance Level] [Power Measurement] …
I ftms: Subscribed to Indoor Bike Data. Pedal!
I ftms: Saved xx:xx:… as the preferred bike
I ftms: Dropped power spike: bike sent up to 456 W, Watch got 135 W
D ftms: 0x2AD2 [8] 44 00 F6 09 AA 00 C8 00             <- raw hex (DEBUG)
I ftms: flags=0x0044 speed=25.50km/h cadence=85.0rpm power=200W   <- decoded (≤1/s at INFO)
W ftms: Bike disconnected (reason=0x208)
I ftms: Rescanning in 1000 ms                           <- backoff 1 s → 2 → 4 → 8 → 10 s max
```

- Raw hex appears only with **Domyos Bridge → DEBUG logging** enabled (off
  by default).
- A preferred bike address is saved in NVS after the first successful
  subscription. For 5 s after each scan starts, other matching bikes are
  ignored so the saved one gets a head start.
- The bike accepts only one connection. If the Domyos app or nRF Connect is
  connected to it, the bike stops advertising and the ESP can't find it.

### Watch side (`cps` and `bridge` tags)

```
W bridge: *** FAKE DATA MODE: sending constant 100 W / 80 rpm (Milestone 1 test) ***
I bridge: BLE address xx:xx:xx:xx:xx:xx (type 0)
I cps:    Advertising as "Domyos Bridge" (name in adv packet)
I cps:    Central connected handle=1 peer=… itvl=30.00ms enc=0 bonded=0
I cps:    Power measurement notifications ON (handle=1, reason=1)
I bridge: bike=connected cadence=80.0 power=100 | CPS: 100 W, revs=53, evt=…/1024s | centrals=1 subscribed=1
```

- The `bridge` INFO summary prints every 5 s. A `(stale)` marker means
  that value is older than the stale timeout, so the bridge is sending 0 W
  with frozen crank counters.
- For per-notification DEBUG lines (and raw FTMS hex from Milestone 2 on),
  enable **Domyos Bridge → DEBUG logging** in menuconfig.
- `reason=0x213` on a disconnect means the remote side closed the link.
  `0x208` means supervision timeout (out of range or powered off).

## Status LED (C3 Super Mini)

On the ESP32-C3 Super Mini, the onboard blue LED is on GPIO8 and is
active-low. Confirm this on your board at first boot: it should light. If
not, change the pin or polarity in menuconfig.

| Pattern | Meaning |
|---|---|
| slow blink (200 ms / 1 s) | looking for the bike |
| solid | bike connected (or fake mode) |
| double blink | bike connected and a Watch subscribed |

## Spec notes and decisions

These are cases where the brief, the Bluetooth specs, and NimBLE needed
reconciling:

- **FTMS "data not available" values.** Per FTMS §4.9.1.10, the
  expended-energy sub-fields use 0xFFFF / 0xFF to mean "not available". The
  parser maps those to `nullopt`. The 5-byte energy group is parsed all or
  nothing.
- **Built-in NimBLE DIS service not used.** In IDF 5.3 its
  manufacturer-name characteristic is compiled out, with no Kconfig switch
  to enable it. 0x180A (manufacturer, model, firmware revision) is defined
  in our own GATT table instead.
- **NimBLE `min`/`max` macros.** NimBLE's `os/os.h` defines function-like
  `min`/`max` macros, which break `<limits>` in C++. All NimBLE includes go
  through `nimble_includes.hpp`, which `#undef`s them.
- **Concurrency.** NimBLE callbacks take a `std::mutex` around a struct
  copy. That is a short, bounded lock, not a blocking call in the
  sleep/wait sense. `ble_gatts_notify_custom` is called from the publish
  task; NimBLE takes its own host lock for that.
- **Central slots.** `BT_NIMBLE_MAX_CONNECTIONS=3`: one for the bike and up
  to two centrals. The second central slot means an iPhone that grabs the
  sensor in the background can't lock the Watch out. Advertising continues
  while a central slot is free.
- **Security.** Bonding is Just Works (NoInputNoOutput, LE Secure
  Connections) with keys in NVS. No characteristic requires encryption, so
  an unbonded Watch works too. If the Watch forgets the device, the stale
  bond on the ESP is deleted automatically (repeat-pairing handler).
- **Crank synthesis timing.** As the brief specifies, each sample's cadence
  applies to the interval since the previous sample. Accumulating in
  `double` means a revolution that lands exactly on a sample boundary can be
  counted one sample late. The tests allow ±1 rev for that case.
- **Power while stopping.** The Domyos reports power one sample after
  cadence: the first packet after you stop has cadence 0 but still carries
  the last pedalling power. The bridge therefore sends 0 W whenever the
  bike's fresh cadence is below 5 rpm (`crank::kStoppedCadenceRpm`, the same
  threshold that freezes the crank counters). A bike that never reports
  cadence is not gated. See `publishable_power_w` in `shared_state.*`.
- **Power spikes.** About once every few minutes, the Domyos sends one update
  (repeated in its ~4 packets, so it lasts ~1 s) with an absurd speed,
  cadence and power, e.g. 135 → 456 → 139 W. These are passed on as-is by
  the CPS spec but ruin peak-power and FTP estimates. `power_filter` holds
  back a rise larger than both 60 W and 50 % of recent power until it has
  lasted 2 s. Real rises in the ride log were at most +56 W/s, so normal
  riding is never delayed; a genuine hard sprint reaches the Watch ~2 s
  late. Drops always pass immediately. Replaying the first ride's log: all
  three spikes dropped, peak 456 W → 151 W. Tunable or switchable under
  **Domyos Bridge → power spike** in menuconfig. Cadence isn't filtered:
  it goes through the crank-revolution count, which already smooths a
  one-second glitch (the Watch's max cadence stayed at 96 rpm).
- **Advertising interval.** NimBLE's defaults (30–60 ms) are used. The
  bridge is USB-powered, so fast advertising costs nothing and helps the
  Watch reconnect quickly.

## Milestones

1. **CPS peripheral, constant 100 W / 80 rpm.** Done: the Watch recorded
   100 W / 80 rpm into Health.
2. **FTMS client: scan/connect/subscribe, log raw and decoded 0x2AD2.**
   Done: it connects to the Domyos (random static address, MTU 247, features
   0x00005687) and decodes the 23-byte, ~4 Hz packets (flags 0x0B76). When
   the bike sleeps (disconnect reason 0x208), the bridge rescans.
3. **Bridge: real cadence and power through `crank_synth` into CPS.** Done:
   the 0 W stop rule triggered on both stops, and the crank counters froze
   through the break and resumed afterwards.
4. Robustness.
   - Done: bike sleep/wake mid-workout. The Watch stayed subscribed and got
     0 W; the bike reconnected within 0.5 s of waking.
   - Left: Watch out of range mid-ride, ESP reboot mid-ride, and starting
     the workout before waking the bike.
