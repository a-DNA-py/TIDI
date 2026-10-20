# T.I.D.I.

*Terrestrial Interface for Dhairya's Intelligence*

My childhood RC car, rebuilt as the body of [GOFI](https://dhairyapandya.com/projects/gofi).
Photos and the story: [dhairyapandya.com/projects/tidi](https://dhairyapandya.com/projects/tidi).

## What's here

| Path | What it is |
|---|---|
| `Firmware/tidi_esp32/` | Everything on the ESP32: drive and steering through a Cytron MDD3A, the two head servos, the TF-Luna LiDAR, buzzer and LED, Bluetooth LE, the UART link to the Pi, a 500 ms failsafe, a wall stop and the room scan |
| `Host/scan_room.py` | Runs a room scan over USB and builds a mesh (`room.obj`), or rebuilds one from a saved CSV |
| `Hardware/wiring.svg` | The schematic |
| `Hardware/esp32.jpg` | The ESP32 dev board used, with its pin labels |

## Run it

The board is an ESP32 dev board (WROOM) on the Arduino core `esp32:esp32` 3.3.12. No extra
libraries: BLE comes with the core.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 Firmware/tidi_esp32
arduino-cli upload  --fqbn esp32:esp32:esp32 -p <port> Firmware/tidi_esp32

pip3 install -r Host/requirements.txt
python3 Host/scan_room.py --port <port> --out room.csv   # scan over USB, then plot
python3 Host/scan_room.py --file room.csv                # plot a saved scan
```

Over USB at 115200, `s` starts a scan and `q` stops it. The wiring is in `Hardware/wiring.svg`, and
the pins are the constants at the top of `tidi_esp32.ino`. The byte layouts are the packed structs
there too.

## The Pi's side

The brain lives in GOFI, which isn't public: its `Node/tidi.py` listens for the wake word with
Whisper, finds faces with OpenCV, asks GOFI, and turns the answer into buzzer notes. It talks to the
ESP32 over UART with the same payloads as Bluetooth LE, framed with a CRC-8. Without it, the car
takes the same commands from any Bluetooth LE client that writes them.

## Known limits

- Bluetooth LE has no pairing. Anything in range that finds the service can connect and drive it.
- There's no fuse on the 3S LiPo lead yet, and the LED has no series resistor. Copy the wiring
  knowing that.

## Licence

MIT. See `LICENSE`.
