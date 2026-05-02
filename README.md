# Vertical Profiling Float

## Python station

Install Python dependencies with uv:

```bash
uv sync
```

Run the topside station after the float prints its IP address in the Arduino Serial Monitor:

```bash
uv run python main.py <FLOAT_IP>
```

Use the interactive menu to configure the mission, zero depth, run manual timed servo movement, start offline depth recording, start the mission, and download recorded data after recovery.

## Arduino dependencies

Install these libraries in Arduino IDE before compiling `master_float/master_float.ino`:

- `WiFiNINA`
- `Servo`
- `BlueRobotics MS5837 Library` or another library that provides `MS5837.h`

In Arduino IDE, use **Tools > Manage Libraries...**, search each name, and install it. The compile error `fatal error: MS5837.h: No such file or directory` means the MS5837 library is missing from the Arduino libraries folder.

## Arduino board settings

- Board: Arduino MKR WiFi 1010
- Servo pins: D2 and D3
- Sensor: MS5837 on I2C
- Servo angle convention: up is 150 degrees, neutral is 90 degrees, down is 30 degrees

## Master firmware

Use `master_float/master_float.ino` for the sealed float. It supports:

- Configurable mission targets, hold time, profile count, P gain, tolerances, and log interval from Python.
- Configurable servo down, neutral, and up angles from Python; defaults are 30, 90, and 150 degrees.
- Surface depth zeroing before mission or depth recording.
- Offline depth recording that can be started topside and downloaded after recovery.
- Timed manual commands: `down <seconds>` and `up <seconds>`, limited to 10 seconds.
- Main mission with two default profiles, then return to surface after the final 0.40 m hold.
- Mission and depth data download as CSV through the Python station.
- One shared 600-sample onboard log buffer for mission or depth-recording data.
- Sensor init tries 10 times; if the MS5837 is unavailable, WiFi and manual servo commands still work, but depth zeroing, depth recording, and mission start return `ERROR NO_SENSOR`.
- Each mission phase has a configurable timeout, defaulting to 180 seconds, so the float does not command one direction forever if a target is never reached.

Set `ssid` and `password` in the Arduino sketch before upload. Mission settings are sent from Python and do not require re-uploading.

Do not power-cycle the float before downloading data. Logs are stored in RAM, not flash.

## Compile troubleshooting

If Arduino IDE reports this error from `WiFiNINA`:

```text
error: 'PinStatus' does not name a type
```

then the sketch is being compiled without the correct SAMD board core. Install/select the MKR WiFi 1010 board support:

1. Open **Tools > Board > Boards Manager...**
2. Search `Arduino SAMD Boards`
3. Install **Arduino SAMD Boards (32-bits ARM Cortex-M0+)**
4. Select **Tools > Board > Arduino SAMD Boards > Arduino MKR WiFi 1010**
5. Compile again

The `Multiple libraries were found for "Servo.h"` message is usually only a warning. The fatal issue is the missing/wrong SAMD board core.

If the compile output says:

```text
may be incompatible with your current board which runs on avr architecture(s)
```

then Arduino IDE is still compiling for an AVR board such as Uno/Nano/Mega. Re-select **Arduino MKR WiFi 1010** under **Tools > Board**. Installing the SAMD package is not enough; the selected board must also be changed.

## Fast WiFi-only test

Use `connectivity_fast_test/connectivity_fast_test.ino` to test the topside client without the pressure sensor or servos attached.

1. Open `connectivity_fast_test/connectivity_fast_test.ino` in Arduino IDE.
2. Set the same `ssid` and `password` values as the main sketch.
3. Select **Arduino MKR WiFi 1010** and upload.
4. Open Serial Monitor and copy the printed IP address.
5. Run:

```bash
uv run python main.py <FLOAT_IP> --output fast_test_data.csv
```

Press Enter in the Python terminal. The test sketch simulates both vertical profiles in about 16 seconds, then the Python client automatically downloads fake `time,depth,state,control` data and plots it.

## Above-ground timed servo test

Use `timed_servo_test/timed_servo_test.ino` to test the WiFi protocol and servo wiring without the pressure sensor or feedback loop.

1. Open `timed_servo_test/timed_servo_test.ino` in Arduino IDE.
2. Set `ssid` and `password`.
3. Connect the servos to D2 and D3.
4. Keep the buoyancy mechanism safe to move above ground.
5. Upload to **Arduino MKR WiFi 1010**.
6. Run:

```bash
uv run python main.py <FLOAT_IP> --output timed_servo_test_data.csv
```

This sketch moves the servos through the full configured range on a timed sequence: 70 degrees for down, 90 degrees for neutral, and 110 degrees for up. It does not read the MS5837 and does not use closed-loop depth control.
