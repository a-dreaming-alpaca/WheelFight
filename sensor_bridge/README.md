# WheelFight sensor bridge

This directory contains the Mega sensor acquisition/communication layer and
its hardware-independent tests. Motor and servo commands remain exclusively in
`../0_FinalSourceCode/motion_controller.py`.

## Components

- `mega_firmware/WheelFightSensors/WheelFightSensors.ino`: samples 15 analog
  and 2 digital inputs and sends versioned, checksummed frames.
- `mega_firmware/DigitalInputScanner/DigitalInputScanner.ino`: temporary
  bring-up sketch that prints the raw level of every safe digital header pin
  from D2 through D53 and lists all pins currently low.
- `../0_FinalSourceCode/mega_sensor_reader.py`: validates frames, tracks link
  health, exposes immutable sensor snapshots, and reconnects after USB serial
  failure. It lives beside the match controller for later integration.
- `../0_FinalSourceCode/sensor_monitor.py`: prints live data and optionally
  records every received frame to CSV.
- `PROTOCOL.md`: the wire protocol and canonical channel map.

## Mega 2560 setup

The firmware depends only on the standard **Arduino AVR Boards** core. No
third-party Arduino library is required.

The default physical pin assignment is:

| Sensor channels | Mega pins |
| --- | --- |
| `A0` ... `A14` | `A0` ... `A14` |
| `DI0` | `D22` |
| `DI1` | `D23` |

The digital pins use `INPUT_PULLUP`, matching active-low sensors and providing
a defined idle level if a signal wire is disconnected. If the selected sensor
module requires a different electrical input mode, change `DIGITAL_PIN_MODE`
in the sketch after checking its output circuit.

In Arduino IDE, select **Arduino Mega or Mega 2560** and processor
**ATmega2560**, then upload the sketch. The configured serial rate is 115200.

### Digital-input scanner

For wiring diagnosis, upload `DigitalInputScanner.ino` instead of the normal
sensor sketch and open the Arduino Serial Monitor at 115200 baud. All D2-D53
pins use `INPUT_PULLUP`; idle inputs should read 1 and an active-low sensor or
a temporary connection to GND should read 0. The final `LOW=` field lists the
pin names that are low, for example `LOW=D22|D24`.

D0/RX0 and D1/TX0 are deliberately excluded because they carry the same USB
serial stream used to print the results. Connecting a sensor to D1 can corrupt
or stop visible serial output. A0-A15 are not reconfigured by this test sketch.

## RK3588S setup

Python 3.9 or newer and `pyserial` are required:

```bash
cd 0_FinalSourceCode
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r sensor_requirements.txt
```

The monitor can discover `/dev/ttyACM*`, `/dev/ttyUSB*`, and entries under
`/dev/serial/by-id` automatically:

```bash
python3 sensor_monitor.py
```

`sensor_monitor.py` is a standalone wiring and acquisition test. Do not run it
at the same time as `match_demo_state_machine.py`, because both processes would
compete for the same Mega serial port. The read-only `tk_monitor.py` is safe to
run alongside the match controller because it only reads the controller's JSON
status file.

The match controller reads its serial read timeout and reconnect interval from
`0_FinalSourceCode/robot_config.py`. The standalone `sensor_monitor.py` keeps
the `MegaSensorReader` diagnostic defaults instead.

An explicit port and CSV recording can also be selected:

```bash
python3 sensor_monitor.py --port /dev/ttyACM0 --csv sensor_run.csv
```

Protocol and parser tests can be run from the repository root without opening
serial hardware:

```bash
python3 -m unittest discover -s sensor_bridge/tests -v
```

If Linux reports a permission error, add the user running the controller to
the distribution's serial-device group (commonly `dialout`), then log in
again. For deployment, prefer the stable device name under
`/dev/serial/by-id` over `/dev/ttyACM0`.

## Bring-up checks

1. Confirm that the link reports about 50 Hz with no CRC or dropped frames.
2. Exercise one sensor at a time and verify the channel map.
3. Verify that each digital sensor changes from 1 to 0 when triggered.
4. Unplug and reconnect USB and confirm automatic recovery.
5. Record at least 30 minutes to CSV before enabling full-power match tests.
