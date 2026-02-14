# Novus Wheel (ODrive + Virtual Steering Wheel)

This adds a new app that turns your real ODrive + encoder wheel into a virtual steering wheel device that games can bind to, and (on Windows) receive force-feedback (FFB) effects that are translated into ODrive torque commands.

Nothing in `scripts/example_spin.py` or `scripts/encoder_live.py` was modified.

## What you get

- **Windows (primary target)**: vJoy virtual device
  - Steering axis output (X axis)
  - Receives DirectInput FFB effects via vJoy (constant force + spring + damper)
  - Translates effects into motor torque using ODrive `TORQUE_CONTROL`

- **macOS (development/limited)**: SDL2 virtual joystick
  - Steering axis only
  - FFB is not available in this backend

## Safety model

- Uses ODrive limits that match `scripts/example_spin.py` defaults:
  - `current_limit_a=16`
  - `vel_limit_turn_s=30`
  - `dc_max_negative_current_a=-5`
- The app *also* clamps torque with `--max-torque-nm` (default `2.0`).
- Any ODrive fault, disconnect, invalid encoder readings, or overspeed triggers a **critical error**:
  - torque command set to 0
  - axis set to IDLE
  - app exits (no auto-restart)

## Windows setup (required for games)

1. Install **vJoy** (driver + config tool). Create/enable a device (usually Device 1) with at least an **X axis**.
2. Ensure `vJoyInterface.dll` is available on PATH (it is installed with vJoy).
3. Ensure the **ODrive USB driver** is set up for the Python tools:
  - The ODrive Python API talks to the ODrive over **libusb**.
  - On Windows this usually requires binding the ODrive **Native Interface** to a **WinUSB** driver.
  - If you see: `[UsbDiscoverer] Failed to open USB device: -5`, it is typically an access/driver issue.
  - Common fix: use **Zadig** to select the ODrive Native Interface and install/replace the driver with **WinUSB**, then unplug/replug the ODrive.
  - Also close the ODrive GUI/other scripts that might be using the device, and try running the terminal as Administrator.
4. Install Python dependencies into your environment (ODrive must be installed in the same venv you run the scripts from).

## macOS setup (optional)

For the SDL backend:

- `pip install pysdl2 pysdl2-dll`

Note: many games on macOS will not see SDL virtual joysticks unless they use SDL.

## Run

From the repo root (or `scripts/`):

```bash
python3 scripts/novus_wheel.py --backend auto
```

Or from the repo root you can also run:

```bash
python3 -m novus_wheel --backend auto
```

Useful options:

```bash
# Windows, pick vJoy device 1 explicitly
python3 scripts/novus_wheel.py --backend vjoy --vjoy-id 1

# Re-run center calibration
python3 scripts/novus_wheel.py --recalibrate

# Set wheel range (lock-to-lock) to 900 degrees = 2.5 turns
python3 scripts/novus_wheel.py --lock-to-lock-turns 2.5

# Limit torque harder
python3 scripts/novus_wheel.py --max-torque-nm 1.0
```

## Calibration

On first run (or with `--recalibrate`), the app will prompt you to center the wheel and press Enter. It stores `novus_wheel_calibration.json` with a center offset.

## Notes / limitations

- On Windows, the device will show up as the vJoy device name (typically `vJoy Device`). The app itself is called **Novus Wheel**.
- vJoy FFB parsing here supports **constant**, **spring**, and **damper** effects. Some games use additional effect types; those are currently ignored.

## Fixing `CPR_POLEPAIRS_MISMATCH`

That error means ODrive's configured encoder CPR doesn't match the motor pole pairs / gearing.
This project supports the same pulley tooth ratio logic as `scripts/odrive_movement.py`:

```bash
# Example: encoder is belt-driven (motor teeth / encoder teeth)
python3 scripts/novus_wheel.py \
  --encoder-cpr 4000 \
  --motor-pulley-teeth 30 \
  --encoder-pulley-teeth 20
```

If you already saved correct values on the ODrive and don't want the app to overwrite them:

```bash
python3 scripts/novus_wheel.py --no-override-encoder-cpr --no-override-motor-pole-pairs
```
