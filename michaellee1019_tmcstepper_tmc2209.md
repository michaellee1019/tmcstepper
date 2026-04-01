# Model michaellee1019:tmcstepper:tmc2209

A motor component for controlling TMC2209 stepper motor drivers via step/direction GPIO pins. Uses libgpiod for direct kernel-level GPIO control, generating step pulses with precise timing via `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`. Each pulse is individually counted for exact position tracking.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "gpio_chip": "<string>",
  "step_pin": <int>,
  "dir_pin": <int>,
  "enn_pin": <int>,
  "steps_per_revolution": <int>,
  "max_rpm": <float>
}
```

### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `gpio_chip` | string | Optional | Name of the GPIO chip device (e.g. `"gpiochip0"`, `"gpiochip4"`). Defaults to `"gpiochip0"`. |
| `step_pin` | int | Required | GPIO line offset for the STEP signal. |
| `dir_pin` | int | Required | GPIO line offset for the DIR signal. |
| `enn_pin` | int | Optional | GPIO line offset for the ENN (enable, active low) signal. When configured, the driver is disabled (ENN HIGH) when the motor is stopped and enabled (ENN LOW) when running. |
| `steps_per_revolution` | int | Required | Number of steps per full revolution. For a typical NEMA stepper this is 200 (1.8 degrees per step). If microstepping is configured via MS1/MS2 hardware pins, multiply accordingly (e.g. 200 * 8 = 1600 for 8x microstepping). |
| `max_rpm` | float | Optional | Maximum allowed RPM. Caps the step frequency for all motion commands. Required if using `SetPower`, since power is interpreted as a fraction of max RPM. |

### Example Configuration

```json
{
  "gpio_chip": "gpiochip4",
  "step_pin": 13,
  "dir_pin": 15,
  "enn_pin": 11,
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
