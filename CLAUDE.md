# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project context

A Python 3 systemd daemon that drives Thermaltake Riing RGB fan/lighting controllers over USB (vendor `0x264a`) via `pyusb`. This is a fork of the abandoned `chestm007/linux_thermaltake_rgb`; see `CHANGES.md` for what this fork changed (new controllers, new lighting models, the `riingplus` alias hack).

## Common commands

Install (from a checkout):

```bash
sudo pip3 install .
# then, on first install, drop the systemd unit and config:
sudo cp /usr/share/linux_thermaltake_rgb/linux-thermaltake-rgb.service /usr/lib/systemd/system/
sudo mkdir -p /etc/linux_thermaltake_rgb/
sudo cp /usr/share/linux_thermaltake_rgb/config.yml /etc/linux_thermaltake_rgb/
```

Run the daemon directly (without systemd) for development; it requires USB access (root or appropriate udev rules):

```bash
sudo python3 run.py
# or, after install:
sudo linux-thermaltake-rgb
```

Tests — CI runs pytest, the in-tree `tests/run_tests.py` uses `nose`:

```bash
python -m pytest                                # CI-equivalent run
python -m pytest tests/unit_tests/test_controllers.py::TestThermaltakeG3Controller   # single test
python tests/run_tests.py                       # legacy nose runner with timing
```

## Architecture

### Plugin discovery via `ClassifiedObject`

`classified_object.py` defines `ClassifiedObject.inheritors()`, which walks `__subclasses__()` recursively. Almost every extension point uses it: controllers, devices, fan models, lighting effects. Each subclass declares a `model` class attribute (a string), and a `factory(model_name, ...)` classmethod looks it up.

Consequence: **a new subclass is only discoverable if its module has been imported.** `devices/__init__.py` does `from linux_thermaltake_rgb.devices.{pumps,fans,lights} import *` for exactly this reason. When adding a new device/effect/model, make sure it lands in a module that gets imported on daemon startup.

### Daemon wiring (`daemon/daemon.py`)

`ThermaltakeDaemon.__init__` does the orchestration in this order:
1. Loads YAML config (`daemon/config.py` — looks at `/etc/linux_thermaltake_rgb/config.yml`, falls back to the in-repo `linux_thermaltake_rgb/assets/config.yml`).
2. Builds a `FanModel` and a `LightingEffect` from the config (both optional — either may be absent).
3. For each controller in the config: instantiates a `ThermaltakeController` subclass, then for each port instantiates a `ThermaltakeDevice` subclass, attaches it to the controller, and registers it with whichever managers care (`ThermaltakeFanDevice` → fan manager; `ThermaltakeRGBDevice` → lighting manager).
4. On stop, calls `controller.save_profile()` to persist the current state to the controller hardware.

### Controller / driver split

- `controllers.py` — high-level `ThermaltakeController` subclasses (`g3`, `riingplus`, `riingtrio`). Each picks a driver from `drivers.py`.
- `drivers.py` — pyusb-level. The base class' `_initialize_device()` resets the device, detaches the kernel driver, claims interface 0, and finds the in/out endpoints.

USB IDs and per-controller constants:
- G3 / RiingPlus: `PRODUCT_ID_BASE = 0x1fa5`, `BY_LED = 0x18` — they share USB IDs and protocol. `ThermaltakeRiingPlusController` is a one-line subclass of `ThermaltakeG3Controller` that just changes `model`; there is no separate RiingPlus driver.
- RiingTrio: `PRODUCT_ID_BASE = 0x2135`, `BY_LED = 0x24`.

Historical note: a `riingquad` controller (`0x2260`) was added in this fork and later removed. The Riing Quad must NOT be sent the `0x32 0x53` save-profile command — it freezes the controller. If you re-add Quad support, override `save_profile` to a no-op in the Quad driver, as the now-removed `ThermaltakeRiingQuadControllerDriver` did.

### `BY_LED` is per-driver, not global

`linux_thermaltake_rgb/globals.py` defines `RGB.Mode.BY_LED = 0x18` (matching G3). Trio needs `0x24`. Custom lighting effects in `lighting_manager.py` must therefore use `device.controller.driver.BY_LED`, not `RGB.Mode.BY_LED`. When adding a new custom effect that issues a per-LED write, follow the same pattern.

### Wire protocol cheatsheet (`globals.py`)

USB writes are 64-byte packets, zero-padded by `_populate_partial_data_array`. First two bytes are command/category:

- `PROTOCOL_GET = 0x33`, `PROTOCOL_SET = 0x32`
- `PROTOCOL_FAN = 0x51`, `PROTOCOL_LIGHT = 0x52`
- Lighting modes (`RGB.Mode`): FLOW=0x00, SPECTRUM=0x04, RIPPLE=0x08, BLINK=0x0c, PULSE=0x10, WAVE=0x14, BY_LED=0x18 (G3) / 0x24 (Trio), FULL=0x19.
- Lighting speeds (`RGB.Speed` and `CustomLightingEffect.{SLOW,NORMAL,FAST,EXTREME}`): bytes `0x03 / 0x02 / 0x01 / 0x00`. (Upstream had these as floats — that was a bug; see `c701a0a`.)

### Adding things

- **A new controller**: add a `ThermaltakeController` subclass in `controllers.py` (with a `model` attribute and an `init` that wires up the driver), and a corresponding `ThermaltakeControllerDriver` subclass in `drivers.py` (set `PRODUCT_ID_BASE` and, if it differs, `BY_LED`). Also extend `controller_factory` if it gets called (the daemon uses `ThermaltakeController.factory` via `ClassifiedObject`, so the factory function is somewhat redundant but kept in sync).
- **A new device**: subclass `ThermaltakeFanDevice`, `ThermaltakeRGBDevice`, or both, in `devices/{fans,lights,pumps}.py`. Set `model` to the exact string used in `config.yml`. For RGB devices set `num_leds` and `index_per_led`.
- **A new lighting effect**: subclass `LightingEffect` (or `CustomLightingEffect` / `ThreadedCustomLightingEffect` for threaded loops) in `lighting_manager.py`. Set `model`, implement `start()` (and `stop()` if threaded). Document it in `README.md`.
- **A new fan model**: subclass `FanModel` in `fan_manager.py` and implement `main()` returning a 0–100 speed.
