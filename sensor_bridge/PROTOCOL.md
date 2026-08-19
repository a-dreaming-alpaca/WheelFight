# WheelFight Mega Sensor Protocol v2

This protocol carries one atomic snapshot of all WheelFight sensors from an
Arduino Mega 2560 to the RK3588S controller.

## Transport

- USB serial, 115200 baud, 8 data bits, no parity, 1 stop bit.
- Mega transmission rate: 50 frames per second.
- One ASCII frame per line, terminated by `\r\n`.
- The RK3588S treats data as stale when no valid frame has arrived for 200 ms.

## Channel map

| Protocol channel | Meaning |
| --- | --- |
| `A0` ... `A11` | Twelve infrared ranging sensors, 30 degrees apart. `A0` points forward and numbering increases clockwise when viewed from above. |
| `A12` | Front underside grayscale sensor. |
| `A13` | Rear underside grayscale sensor. |
| `A14` | Rear high-mounted infrared ranging sensor. |
| `DI0` | Front-left downward-looking photoelectric sensor. Active low. |
| `DI1` | Front-right downward-looking photoelectric sensor. Active low. |

The digital fields always contain the raw electrical level (`0` or `1`). The
receiver exposes separate active-low boolean properties so the raw evidence is
not lost.

The grayscale and rear high-ranging fields are also transmitted as raw ADC
values. Current grayscale measurements are about 300 on the platform and 900
off it; polarity, filtering, and hysteresis are interpreted only by the RK3588S
perception layer.

## Frame format

```text
<payload>*<crc16>\r\n
```

The comma-separated payload is:

```text
WF2,sequence,mega_millis,A0,A1,...,A14,DI0,DI1
```

A complete example frame is:

```text
WF2,42,123456,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,0,1*838C
```

- `WF2`: protocol and version marker.
- `sequence`: unsigned 32-bit frame sequence, incremented after every frame.
- `mega_millis`: unsigned 32-bit Mega `millis()` value captured at sampling.
- `A0` ... `A14`: raw 10-bit ADC readings in the range 0 ... 1023.
- `DI0` ... `DI1`: raw digital readings, each either 0 or 1.
- `crc16`: four uppercase hexadecimal digits.

CRC is CRC-16/CCITT-FALSE over the ASCII bytes of `<payload>` only:

- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- Reflect input/output: false
- Final XOR: `0x0000`

Frames with a bad marker, field count, numeric range, or CRC are discarded in
full. Protocol v2 has no commands from the RK3588S to the Mega.
