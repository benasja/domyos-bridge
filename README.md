# Domyos Bridge

My Decathlon Domyos bike measures power and cadence, and my Apple Watch can
record power from a Bluetooth power meter. They just don't speak the same
protocol: the bike uses FTMS, and the Watch only understands Cycling Power
sensors. So my Indoor Cycle workouts had heart rate and nothing else.

This is firmware for a 3 € ESP32 that sits next to the bike and translates.
It reads the bike over Bluetooth and shows up on the Watch as an ordinary
power meter. Power and cadence end up in Apple Health, and no phone app has
to stay open during the ride.

It should work with any indoor bike that speaks Bluetooth FTMS (most bikes
that work with Zwift or Kinomap), but I've only ridden it on a Domyos. If
you try another bike, please open an issue either way.

## Install

**[Open the browser installer](https://benasja.github.io/domyos-bridge/)**
in Chrome or Edge on a computer, plug in the ESP32 with a USB data cable and
press Install. There are no settings to enter.

Any ESP32-C3, ESP32-S3 or classic ESP32 works. I use an ESP32-C3 Super Mini.
If the installer can't see the board, hold BOOT while plugging it in.

## Pair and ride

1. Power the board near the bike and wake the bike. The bridge finds it and
   remembers it. On a C3 Super Mini the LED goes from a slow blink to solid.
2. On the Watch (watchOS 10+): **Settings → Bluetooth → Health Devices →
   Domyos Bridge**. Accept the pairing if asked; there's no PIN.
3. Start **Workout → Indoor Cycle**. If power and cadence don't appear, add
   them under the workout's **⋯ → Workout Views**.

Close any other app connected to the bike first (Kinomap, the Domyos app,
Zwift). The bike accepts one connection, and while something else holds it,
it stops advertising and the bridge can't find it.

| LED (C3 Super Mini) | Meaning |
|---|---|
| slow blink | looking for the bike |
| solid | bike connected |
| double blink | bike connected and the Watch is listening |

## Why not just use an app?

[QZ](https://github.com/cagnulein/qdomyos-zwift) can do this from an iPhone,
and much more. I wanted something that runs without a phone in the loop: the
bridge is always on, the Watch connects to it by itself, and a workout is
just "start Indoor Cycle".

## Fixes on the way through

- **Power spikes.** Every few minutes my Domyos reports one absurd value
  (135 → 456 → 139 W), which wrecks peak power and the Watch's FTP estimate.
  The bridge holds back a sudden jump until it has lasted 2 seconds. Normal
  riding is never delayed.
- **Power after you stop.** The bike keeps reporting your last power for a
  sample after you stop pedalling. The bridge sends 0 W once cadence drops.
- **Bike going to sleep.** If the bike sleeps mid-workout, the Watch stays
  connected and gets 0 W. The bridge reconnects within a second of the bike
  waking.

## Build it yourself

ESP-IDF v5.3:

```bash
idf.py set-target esp32c3     # or esp32s3, esp32
idf.py build
idf.py -p PORT flash monitor
```

`idf.py menuconfig` → **Domyos Bridge** has the names it advertises, the
spike filter thresholds, a fake-data mode for testing the Watch side without
a bike, the LED pin, and debug logging.

The protocol parsing and the maths have host unit tests that need only a
C++17 compiler and CMake:

```bash
cmake -S test -B build-test -G Ninja && cmake --build build-test && ctest --test-dir build-test
```

[docs/how-it-works.md](docs/how-it-works.md) has the details: the code
layout, the spec corner cases, how to read the logs, and what's not tested
yet.

## License

MIT
