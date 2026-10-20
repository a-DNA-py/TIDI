# Bill of materials

Everything on the car. How it connects is in [`wiring.svg`](wiring.svg).

## The car

| Part | Qty | Job |
|---|---|---|
| RC car chassis with its original brushed drive and steering motors | 1 | The body. Both motors are rated 9 V |
| ESP32 dev board, ESP32-WROOM-32, 30 pins | 1 | Runs `Firmware/tidi_esp32`. Pin labels: [`esp32.jpg`](esp32.jpg) |
| Cytron MDD3A dual motor driver | 1 | Drive motor on M2, steering on M1. 4–16 V, 3 A continuous, 5 A peak for under 5 s |
| 3S LiPo, 11.1 V (12.6 V full), 1000 mAh, 40C | 1 | The only battery on the car |
| Switch | 1 | On the LiPo's positive lead |
| XH-M404 buck module (XL4016E1), set to 5 V | 1 | The 5 V rail for the ESP32, both servos and the TF-Luna |
| TF-Luna LiDAR | 1 | Distance, over UART at 115200: the room scan and the wall stop |
| MG90S servo, 180° | 2 | The LiDAR head: one turns it (azimuth), one tilts it |
| Passive buzzer, two wires | 1 | TIDI's voice, driven straight from GPIO22 |
| LED | 1 | Lights with the voice, on GPIO23 |
| Resistor, 10 kΩ | 1 | Battery voltage divider into GPIO34 |
| Resistor, 4.7 kΩ | 2 | The divider's lower leg, in parallel (2.35 kΩ) |
| 3S LiPo balance charger | 1 | Off the car |

## The brain

These run [GOFI](https://dhairyapandya.com/projects/gofi), whose code isn't in this repo.

| Part | Qty | Job |
|---|---|---|
| Raspberry Pi 5, 8 GB | 1 | Talks to the ESP32 over UART (its GPIO14/15 to the ESP32's GPIO16/17) |
| UPS HAT (E) for the Pi 5 | 1 | Powers the Pi on its own batteries, separate from the car's rails |
| Philips SPC 1000NC USB webcam | 1 | Faces forward: the camera, and the microphone that hears the wake word |
| NEO-6M GPS module | 1 | Location, on the Pi's UART2 (GPIO4/5) at 3.3 V. Wired; the Pi doesn't read it yet |

## Power

The motors run from the LiPo through the MDD3A, and the firmware caps the PWM duty so they see about
7 V on average. The drive motor draws about 3 A and the steering motor 4–5 A stalled at their rated
9 V. The MDD3A gives 5 A for under 5 s, so the firmware cuts steering held longer than 3 s. At about 3 A of driving, the 1000 mAh
pack lasts roughly 15 minutes (an estimate, not measured).

The buck rail carries two MG90S at about 0.4 A each when stalled
([datasheet](https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf)), plus the ESP32 and
the TF-Luna. The XL4016E1 is rated 8 A
([datasheet](https://www.alldatasheet.com/datasheet-pdf/pdf/1134370/XLSEMI/XL4016E1.html)); the
module is sold as 5 A continuous without a fan
([listing](https://robu.in/product/xl4016e1-200w-step-down-power-supply-module/)). It needs at least
8 V in, and the firmware's 9.9 V low-battery cutoff stays above that.

There's no fuse on the LiPo lead yet, and the LED has no series resistor: the firmware sets its pin
to the weakest drive strength instead.
