# Changes in this fork

This fork picks up from upstream `chestm007/linux_thermaltake_rgb` at commit
`9d36f2c` (Mar 2024), after which upstream went quiet. The original code only
recognized two controllers, `g3` and `riingtrio`.

## Bootstrap fixes

- `1afd504` — `setup.py` fixes for Fedora 39: `GObject` → `PyGObject`, and a
  real version (`0.2.0`) in place of the literal placeholder `PROJECTVERSION`.
- `c701a0a` — fix `CustomLightingEffect` speed constants in
  `lighting_manager.py`: the upstream values were floats
  (`1 / 0.75 / 0.5 / 0.25`); replaced with bytes (`0x03 / 0x02 / 0x01 / 0x00`).
  The USB protocol expects a byte here, so the float values could not be sent
  to the device.

## New lighting models

- `ee8ccbf` — `Temperature2LightingEffect`: an alternative temperature-driven
  lighting model that interpolates between two RGB colors across a temperature
  range. Includes a config example. README description added in `9dc97d3`.
- `7b8c93a` — `TimeOfDayLightingEffect`: a time-of-day lighting model. Pulls
  in `python-dateutil`; the dependency was formally declared later in
  `f103023`. README markup fix for the related clock model in `6ea654b`.
- `212f471` — added `build/` to `.gitignore`.

## Riing Quad support (added, then removed)

- `ed4daca` "preliminary Riing Quad controller support":
  - New `ThermaltakeRiingQuadController` / `ThermaltakeRiingQuadControllerDriver`
    (USB product ID base `0x2260`).
  - Introduced a per-driver `BY_LED` constant (G3 = `0x18`, Trio = `0x24`,
    Quad = `0x24`) and changed every custom effect (`Alternating`,
    `Temperature`, `Temperature2`, `Clock`, `PerLED`) to use
    `device.controller.driver.BY_LED` instead of the hard-coded
    `RGB.Mode.BY_LED`. This is what makes per-LED effects work on Trio (and
    formerly Quad).
  - Disabled the `save_profile` write (`0x32 0x53`) on the base driver class
    because it freezes a Riing Quad. The G3 driver keeps issuing it; the Quad
    driver has its own no-op override.
- The Quad classes were later removed from this fork (no longer using that
  hardware). The per-driver `BY_LED` constant and the `save_profile` split
  remain, since they still benefit Trio and G3.

## Debugging + the `riingplus` alias

- `8ac34a7` — added `LOGGER` debug messages around USB writes in `drivers.py`.
- `a6009e9` — **the "this works as magic for me" hack.** Added a `riingplus`
  controller that was a near-clone of `g3`: same `PRODUCT_ID_BASE = 0x1fa5`,
  same `BY_LED = 0x18`. The Riing Plus hardware speaks the G3 protocol and
  shares the G3's USB product ID, so the alias was effectively a rename.
  This commit also re-enabled `save_profile` on the base class (safe now,
  since the Quad driver has its own no-op override).
- `f103023` — declared `python-dateutil` in `setup.py` (used by the
  time-of-day model).

## Why the alias actually mattered (post-mortem)

`type: g3` in the YAML did not work for `unit > 1`, but `type: riingplus`
did. The reason is a long-standing upstream bug in
`ThermaltakeG3Controller.__init__` (introduced in upstream `6c5eeb5`,
2019): it called `super().__init__()` without forwarding `unit`, so the
base class ran `init()` with the default `unit=1` and constructed a driver
for product `0x1fa5` (initializing USB and claiming the interface), then
the G3 constructor built a *second* driver for the actual unit. For
`unit=1` this happened to work; for `unit=2` it claimed the wrong device
first.

`ThermaltakeRiingPlusController` did not reproduce that bug — it only
overrode `init()` and let the base class carry `unit` through correctly.
That is why the alias "worked as magic." Later cleanup deleted the broken
`__init__` from `ThermaltakeG3Controller` and turned
`ThermaltakeRiingPlusController` into a true alias (a one-line subclass of
the G3 controller); the duplicate `ThermaltakeRiingPlusControllerDriver`
was removed.

### Why this stayed hidden upstream for ~7 years

- **Single-controller setups don't trigger it.** With `unit=1` (the
  default), the wasted first driver build hits the same USB device as the
  rebuild. The doubled init is silent — no functional difference, no
  user-visible symptom. Most users own one G3.
- **Multi-G3 users are rare**, and when the bug does fire the symptom is
  flaky USB state at startup, not a hard error. Easy to misread as a udev
  issue, the kernel HID driver re-attaching, or "Thermaltake hardware
  being weird."
- **The fork's own author worked around it without realizing.** Commit
  `a6009e9`'s message ("this works as magic for me") is literal: switching
  the YAML from `type: g3` to `type: riingplus` routed instantiation
  through a class with no `__init__` override, dodging the bug. The
  workaround stuck because it worked, so the cause stayed buried.
- **Tests don't catch it.** `test_controller_factory` constructs with
  `_initialize_device` mocked, so the wrong-device USB claim is a no-op.
  Even an explicit "construct unit 2" test would have passed.
- **Upstream went quiet in 2019** (last PyPI upload 2019-07-28; AUR
  package last touched 2019-03-06), so there was no maintainer left to
  receive a bug report even if a multi-G3 user had filed one.
