Using the cc3100 network driver
==================

## Connecting the CC3100BOOST board to the pyboard
At this point, the driver assumes the following signal connections between the Pyboard and the CC3100BOOST board:

Pyboard | CC3100BOOST board
--------|-------------------
PB15/Y8 (MOSI) | DIN
PB14/Y7 (MISO) | DO
PB13/Y6 (SCK)  | CLK
PB12/Y5 (CS)   | CS
PB9/Y4  (EN)   | nHIB
PB8/Y3         | IRQ

The CC3100 board can tap USB power from the pyboard, but not from the pyboard's 3.3V regulator; this regulator cannot provide sufficient current to power both the pyboard and the CC3100.

To power the CC3100 board from the pyboard's USB supply, connect the 5V pin of the CC3100 board to the VIN pin of the pyboard and ensure that there is a good connection between Ground (GND) pins on both boards. Then move jumper J8 on the CC3100 board from the 'MCU' position to the 'LDO' position; this will allow the regulator on the CC3100 board to draw power from the 5V USB rail. To help ensure stability, you can also put a capacitor (10 to 100uF) between the GND and 5V lines connecting the pyboard and the CC3100 board.

## Building the source
The following command can be used to build the source and deploy directly to the pyboard:
```make -B MICROPY_PY_CC31K=1 USE_PYDFU=1 deploy```
The firmware will be deployed using `dfu-util` if the `USE_PYDFU=1` option is omitted.

## Using the driver
Note that the wifi interface initialization requires the SPI port and pin arguments, although at this point they are ignored within the driver (TODO: fix code to utilize pin/port args)
Module has successfully connected to WPA and WPA2 networks, I have been unable to connect to a WEP network. (blmorris)

```python
import network
import socket
ssid = <wireless network name>
password = <wireless network password>
wifi = network.CC31k(pyb.SPI(2), pyb.Pin.board.Y5, pyb.Pin.board.Y4, pyb.Pin.board.Y3)
pyb.delay(100)
wifi.connect(ssid,key=password)
while (not wifi.is_connected()):
    wifi.update()
    pyb.delay(100)

print(wifi.is_connected())

# Now we can create a new network socket:
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(s)
```
`print(s)` should return `<socket>`
