# NRF51-serial-multitool
UART to BLE adapter working with SpeedyBee, VESC and even APA102

## Protocols
### UART
if [USE_UART](Makefile#L199) is enabled in the Makefile, the adapter provides
*HM-10*-style characterstics to talk to, which apps like *SpeedyBee* use to speak to flight controllers.
Also, there is a *Nordic UART Service* with both the RX and TX characteristic, that works in parallel with the HM-10 service.
Apps like *VESC Tool* use that style to talk to VESC's.
Currently, only 115200 baud is supported, but this can be changed through code.

### SPI
if [USE_SPI](Makefile#L198) is enabled in the Makefile, the adapter can be used to (only) output SPI data.
I implemented this to control APA102 LED's on a low level, since the understand the SPI protocol.

### DFU
if [USE_DFU](Makefile#L197) is enabled in the Makefile, the application can be updated using *nRF Connect* on android (and maybe on ios) with the .zip
provided in future releases.

## Hardware
I used a board called wt51822-s4at since it is the smallest
BLE board I could find.
It has 256kb flash and 16kb ram.
Other boards can be used of course, the pins may need adjustments.
### Pins
On the boards I used, the [following layout](boards/WT51822_S4AT.h#L25) is in place:

RX   -> P0.1\
TX   -> P0.2\
SCK  -> P0.3\
MOSI -> P0.4

The board is called "Serial adapter" by default, but can be renamed using *nRF Connect* or any other tool that allows writing the device name characteristic.

## Installation
You will need some sort of JLink debugger probe.
I only tested with SEGGER JLink Edu, a 20€ probe working really well.
After hooking up your device you'll have to decide on whether you want future updates over the air.
All the pre-compiled files can be found [here](https://github.com/dakhnod/NRF51-serial-multitool/releases).
### With DFU updates
You'll have to install the bootloader first,
then connected to the device "DfuTarg" and push the zip
application_only.zip.
```
nrfjprog --recover
nrfjprog --program bootloader_with_softdevice.hex --sectorerase
nrfjprog -r
```
After that, you can flash application_only.zip through nRF Connect.

### Without DFU updates
This is more straightforward since it skips the bootloader.
Doesn't allow for updates over the air, though.
```
nrfjprog --recover
nrfjprog --program application_with_softdevice.hex --sectorerase
nrfjprog -r
```