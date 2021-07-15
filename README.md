# NRF51-serial-multitool
UART to BLE adapter working with SpeedyBee, VESC and even APA102

## Protocols
### UART
if *USE_UART* is enabled in the Makefile, the adapter provides
*HM-10*-style characterstics to talk to, which apps like *SpeedyBee* use to speak to flight controllers.
Also, there is a *Nordic UART Service* with both the RX and TX characteristic, that works in parallel with the HM-10 service.
Apps like *VESC Tool* use that style to talk to VESC's.

### SPI
if *USE_SPI* is enabled in the Makefile, the adapter can be used to (only) output SPI data.
I implemented this to control APA102 LED's on a low level, since the understand the SPI protocol.

### DFU
if *USE_DFU* is enabled in the Makefile, the application can be updated using *nRF Connect* on android (and maybe on ios) with the .zip
provided in future releases.

## Hardware
I used a board called wt51822-s4at since it is the smallest
BLE board I could find.
It has 256kb flash and 16kb ram.
Other boards can be used of course, the pins may need adjustments.
### Pins
On the boards I used, the following layout is in place:

RX   -> P0.1\
TX   -> P0.2\
SCK  -> P0.3\
MOSI -> P0.4

The board is called "Serial adapter" by default, but can be renamed using *nRF Connect* or any other tool that allows writing the device name characteristic.