# Model michaellee1019:tmcstepper:tmc2209

A motor component for controlling TMC2209 stepper motor drivers via step/direction GPIO pins. Uses libgpiod for direct kernel-level GPIO control, generating step pulses with precise timing via `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`. Each pulse is individually counted for exact position tracking.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "step_pin": "<string>",
  "dir_pin": "<string>",
  "enn_pin": "<string>",
  "steps_per_revolution": <int>,
  "max_rpm": <float>
}
```

Omit `gpio_chip` on Raspberry Pi boards to **auto-detect** the GPIO device (tries common `/dev/gpiochip*` paths in order). Set `gpio_chip` only when you need a specific device.

Pin attributes are **strings** (decimal line offsets, e.g. `"17"`). Numbers are still accepted for compatibility, but strings are recommended.

### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `gpio_chip` | string | Optional | GPIO character device: basename (e.g. `"gpiochip0"`) or full path. If the name does not start with `/`, `/dev/` is prepended. **If omitted or empty**, the module tries, in order: `/dev/gpiochip0` (typical Pi 3/4), `/dev/gpiochip10`, `/dev/gpiochip11` (Pi 5 header on many images), then `/dev/gpiochip4` and `/dev/gpiochip1`. The first path that opens successfully is used. |
| `step_pin` | string | Required | GPIO line offset for the STEP signal (string decimal, e.g. `"27"`). |
| `dir_pin` | string | Required | GPIO line offset for the DIR signal. |
| `enn_pin` | string | Optional | GPIO line offset for the ENN (enable, active low) signal. When configured, the driver is disabled (ENN HIGH) when the motor is stopped and enabled (ENN LOW) when running. |
| `steps_per_revolution` | int | Required | Number of steps per full revolution. For a typical NEMA stepper this is 200 (1.8 degrees per step). If microstepping is configured via MS1/MS2 hardware pins, multiply accordingly (e.g. 200 * 8 = 1600 for 8x microstepping). |
| `max_rpm` | float | Optional | Maximum allowed RPM. Caps the step frequency for all motion commands. Required if using `SetPower`, since power is interpreted as a fraction of max RPM. |

### Raspberry Pi (3 / 4 / 5 / Zero 2 W)

Leave **`gpio_chip` unset** for automatic selection across common Pi images. GPIO **line numbers** (`step_pin`, `dir_pin`, etc.) are offsets on the chosen chip; they differ between Pi 4 (BCM) and Pi 5 (RP1), so use the mapping for your hardware (e.g. `gpioinfo` on the robot).

If opening the chip fails:

1. Run `ls /dev/gpiochip*` and, if needed, set `gpio_chip` explicitly to that basename or full path.
2. In **Docker**, pass every device you might need, e.g. `--device /dev/gpiochip0 --device /dev/gpiochip10`, or the paths your Pi actually exposes.
3. Ensure the process can read `/dev/gpiochip*` (often `dialout` group or root).

### Example Configuration

```json
{
  "step_pin": "17",
  "dir_pin": "27",
  "enn_pin": "22",
  "steps_per_revolution": 1600,
  "max_rpm": 300
}
```

To force a specific chip (e.g. pinned deployment):

```json
{
  "gpio_chip": "gpiochip0",
  "step_pin": "17",
  "dir_pin": "27",
  "steps_per_revolution": 1600,
  "max_rpm": 300
}
```

## Supported Methods

| Method | Description |
|---|---|
| `SetPower` | Set motor speed as a fraction of `max_rpm` (-1 to 1). Requires `max_rpm` to be configured. |
| `SetRPM` | Spin the motor indefinitely at the given RPM. |
| `GoFor` | Move a specific number of revolutions at a given RPM. Blocks until complete. Passing 0 revolutions behaves like `SetRPM`. |
| `GoTo` | Move to an absolute position (in revolutions from zero) at a given RPM. Blocks until complete. |
| `Stop` | Stop the motor immediately and disable the driver (if ENN is configured). |
| `GetPosition` | Return the current position in revolutions (exact pulse count). |
| `ResetZeroPosition` | Set the current position as the new zero, with an optional offset. |
| `GetProperties` | Reports that position reporting is supported. |
| `IsMoving` | Returns whether the motor is currently in motion. |
| `GetPowerStatus` | Returns on/off status and power percentage (0 or 1 for steppers). |
