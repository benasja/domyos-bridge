# How it works

```
bike ──FTMS (0x1826)──▶ ESP32 ──Cycling Power (0x1818)──▶ Apple Watch
```

The ESP32 is a Bluetooth central and peripheral at the same time. As a
central it connects to the bike and subscribes to Indoor Bike Data (0x2AD2).
As a peripheral it advertises a Cycling Power service and sends one Cycling
Power Measurement (0x2A63) per second with power and crank revolutions. The
Watch treats it like any power meter.

Cycling Power has no cadence field. Cadence is sent as a cumulative crank
revolution count plus the time of the last revolution, and the Watch works
cadence out from that. The bike only reports rpm, so `crank_synth` turns rpm
over time into those counters.

## Code layout

```
main/
  main.cpp            NVS and NimBLE init, 1 Hz publish task
  ftms_parser.*       pure: FTMS 0x2AD2 / 0x2ACC bytes -> structs
  ftms_client.*       NimBLE central: scan, connect, discover, subscribe, reconnect
  crank_synth.*       pure: cadence over time -> crank revs + event time
  power_filter.*      pure: holds back one-update power spikes
  shared_state.*      latest bike data behind a std::mutex (no IDF)
  cps_server.*        NimBLE peripheral: CPS + Device Information GATT, advertising
  status_led.*        optional LED (GPIO8 on the C3 Super Mini)
  nimble_includes.hpp NimBLE headers, with its min/max macros undone
  Kconfig.projbuild   the "Domyos Bridge" menuconfig menu
test/                 host unit tests, plain CMake and a 40-line test runner
```

The "pure" files have no ESP-IDF dependency, which is why they can be unit
tested on a laptop.

## Things that weren't obvious

- **FTMS "not available" values.** The expended-energy fields use 0xFFFF /
  0xFF for "not available" (FTMS 4.9.1.10). The parser turns those into
  `nullopt`, and parses the 5-byte energy group all or nothing.
- **Device Information.** IDF 5.3's built-in NimBLE DIS service compiles out
  the manufacturer name with no Kconfig switch for it, so 0x180A is defined
  in the bridge's own GATT table.
- **NimBLE's `min`/`max` macros** break `<limits>` in C++. Every NimBLE
  include goes through `nimble_includes.hpp`, which `#undef`s them.
- **Connection slots.** `BT_NIMBLE_MAX_CONNECTIONS=3`: the bike plus two
  centrals. The spare slot means an iPhone grabbing the sensor in the
  background can't lock the Watch out. Advertising continues while a slot is
  free.
- **Security.** Just Works bonding (LE Secure Connections), keys in NVS. No
  characteristic needs encryption, so an unbonded Watch works too. If the
  Watch forgets the device, the stale bond on the ESP is deleted on the next
  pairing attempt.
- **Power while stopping.** The Domyos reports power one sample after
  cadence: the first packet after you stop pedalling still carries the last
  power. The bridge sends 0 W whenever fresh cadence is under 5 rpm, the same
  threshold that freezes the crank counters. Bikes that never report cadence
  aren't gated (`publishable_power_w` in `shared_state.*`).
- **Power spikes.** Every few minutes the Domyos sends one update with an
  absurd speed, cadence and power (135 → 456 → 139 W), repeated over its ~4
  packets so it lasts about a second. Passed through, it wrecks peak power and
  the FTP estimate. `power_filter` holds back a rise bigger than both 60 W
  and 50 % of recent power until it has lasted 2 s. The biggest real rise in
  my ride logs was +56 W/s, so normal riding is never delayed; a real sprint
  reaches the Watch about 2 s late. Drops always pass straight through.
  Replaying my first ride: all three spikes removed, peak 456 W → 151 W. The
  thresholds are in menuconfig, and the filter can be turned off.
- **Crank timing.** Each sample's cadence applies to the interval since the
  previous sample. A revolution that lands exactly on a sample boundary can
  be counted one sample late; the tests allow ±1 rev for that.
- **Preferred bike.** After the first successful subscription the bike's
  address is saved. For 5 s after each scan starts, other bikes are ignored
  so the saved one gets first go.
- **Advertising interval.** NimBLE's default 30–60 ms. The board is on USB
  power, so fast advertising costs nothing and the Watch reconnects quickly.

## Reading the logs

Open the serial monitor (`idf.py -p PORT monitor`, or the "Logs" option in
the browser installer).

Bike side (`ftms`):

```
I ftms: Scanning for an FTMS bike (0x1826 or name "Domyos-Bike*")
I ftms: Connecting to bike xx:xx:xx:xx:xx:xx (type 0, rssi -55)
I ftms: Fitness Machine Feature: features=0x…  target_settings=0x…
I ftms:   bike claims: [Cadence] [Total Distance] [Resistance Level] [Power Measurement] …
I ftms: Subscribed to Indoor Bike Data. Pedal!
I ftms: Saved xx:xx:… as the preferred bike
I ftms: Dropped power spike: bike sent up to 456 W, Watch got 135 W
I ftms: flags=0x0044 speed=25.50km/h cadence=85.0rpm power=200W
W ftms: Bike disconnected (reason=0x208)
I ftms: Rescanning in 1000 ms          <- backoff 1, 2, 4, 8, then 10 s
```

Watch side (`cps`, `bridge`):

```
I cps:    Advertising as "Domyos Bridge"
I cps:    Central connected handle=1 peer=… itvl=30.00ms enc=0 bonded=0
I cps:    Power measurement notifications ON (handle=1, reason=1)
I bridge: bike=connected cadence=80.0 power=100 | CPS: 100 W, revs=53, evt=…/1024s | centrals=1 subscribed=1
```

- The `bridge` summary prints every 5 s. `(stale)` means that value is
  older than the stale timeout, so the Watch is getting 0 W and frozen crank
  counters.
- Disconnect `reason=0x213`: the other side closed the link. `0x208`:
  supervision timeout (out of range or switched off).
- **Domyos Bridge → DEBUG logging** in menuconfig adds the raw 0x2AD2 hex
  and a line for every notification.

## Testing without a Watch

With nRF Connect on a phone: connect to "Domyos Bridge", open Cycling Power
and enable notifications on 0x2A63. You should see
`20-00-PP-PP-RR-RR-TT-TT` once a second (flags 0x0020, power, crank revs,
event time). Disconnect it afterwards, or it holds one of the two slots.

**Domyos Bridge → Fake data mode** ignores the bike and sends a constant
100 W / 80 rpm, which is handy for checking the Watch side alone.

## What's tested and what isn't

Tested on real rides: a 17-minute Indoor Cycle workout with a stop, the bike
going to sleep and waking mid-workout (the Watch stayed subscribed and got
0 W; the bike reconnected within 0.5 s of waking), and the spike filter on
recorded rides.

Not tested yet: the Watch going out of range mid-ride, the ESP rebooting
mid-ride, and starting the workout before waking the bike.
