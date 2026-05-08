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
  controller that is effectively a clone of `g3`: same
  `PRODUCT_ID_BASE = 0x1fa5`, same `BY_LED = 0x18`. The Riing Plus hardware
  speaks the G3 protocol and shares the G3's USB product ID, so configuring
  the device as `g3` was already working — the alias just lets the config YAML
  say `riingplus`. This commit also re-enabled `save_profile` on the base
  class (safe now, since the Quad driver has its own no-op override).
- `f103023` — declared `python-dateutil` in `setup.py` (used by the
  time-of-day model).
