# Description

<img src="https://github.com/Lahorde/airbeam/blob/AirBeam_CairsensN02/img/airbeam_cairsens_fixation.JPG?raw=true" width="700">

Add NO2 sensor to AirBeam. We use [Cairclip Sensor A40-0020-B](http://cairpol.com/fr/savoir-faire-systemes-autonomes-pour-la-surveillance-de-polluants-a-faible-concentration/produits-cairnet-cairtube-cairclip-capteur-cairsens/cairclip-solution-ultra-portable/) 

Here is an article about [Cairsens N02 data quality](http://www.mdpi.com/2504-3900/1/4/473/pdf)

# Connect Cairsens NO2 sensor
Cairclip sensor must be powered to 5V. It uses an UART @9600bps to communicate. Refer doc in `./doc/datasheet/CairClipUart-Interface-Protocole-downloadDataOnly.211116.pdf`

Cairclip sensor can be either connected to AirBeam using an USB/Serial converter or directly connected to some AirBeam GPIO Pins. In all cases, UART HIGH logical level must be 3.3V. Either an hardware UART or a software UART can be used to communicate with it. 

AirBeam MCU is an ATmega32U4, here is PINOUT :

<img src="https://www.arduino.cc/en/uploads/Hacking/32U4PinMapping.png" width="500">

Its associated Arduino name is [Leonardo](https://www.arduino.cc/en/Guide/ArduinoLeonardoMicro) 
ATmega32U4 has 2 hw serial ports :
* virtual Serial port `Serial`, an USB CDC used to communicate with computer over USB connection
* physical UART, on pins D0/RX D1/TX, `Serial1` on AirBeam these pins are left unconnected. 
* some software UARTs can be used on some PINS, RX must be : 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI). Now, Bluetooth chip already uses a software UART on PINS Rx/10 Tx/11. If we instantiate another SoftwareSerial, it is not possible to handle simultaneously two UART Rx on these ports. Single software active Rx Serial must be selected calling `my_serial_instance.listen()`. 

AirBeam ATmega32U4 logical level is 5V :
* a 3.3V/5V level conversion must be done, either using a [5V/3.3V bidirectional level converter](https://hackspark.fr/fr/3-3v-5v-logic-level-converter.html) or a voltage divider network. 

## Connect using Serial1
ATmega32U4 are left unconnected on AirBeam. We can use it but it requires some soldering. 
After, logical level conversion must be done using a 3.3V/5V level converter. 

Here are detailed steps for wiring :
* solder an angled 2*3 connector to CON_ICSP
* add a female wire from CON_ICSP 5V and solder it to logical converter +5V
* solder wire from 3.3V BT pad to logical converter +3.3V
* add a female wire from CON_ICSP GND and solder it to logical converter GND
* solder a wire from ATmege32U4 D1/Tx to logical converter high voltage channel 0
* solder a wire from ATmege32U4 D0/Rx to logical converter high voltage channel 1
* solder a wire logical converter low voltage channel 0 to a female wire (green) going on CON_EXTERN 
* solder a wire logical converter low voltage channel 1 to a female wire (white) going on CON_EXTERN
* prepare an USB-Mini cable, cut it and connect its wires to some male pins
* connect USB green wire to female green wire (Cairsens Rx/Airbeam Tx)
* connect USB white wires to female white wire (Cairsens Tx/Airbeam Rx)
* connect USB black wire GND to CON_EXTERN GND
* connect USB red wire (Cairsens 5V power supply) to CON_EXTERN +5V

Here is an Airbeam picture after these modifications have been done :
<img src="https://github.com/Lahorde/airbeam/blob/AirBeam_CairsensN02/img/airbeam_cairsens.jpg?raw=true" width="500">
    
## Connect using USB / Serial converter
Connect an USB/Serial converter to AirBeam microUSB. Select a [converter with 3.3V logical level](https://www.digitalsmarties.net/products/usb-bub). On the other side of the converter, connects it to Cairclip sensor. 
This solution requires an USB micro male / USB mini male, an USB mini male / 2.5mmx4 male connectors.
It's not a convenient solution :
* Cairclipe sensor must be disconnected when flashing new firmware
* no more Serial logs

# Get measure in AirCasting application
In adition to other :
* humidity
* temperature
* Pm2.5
* Sound level

You will add NO2 concentration in AirCasting application :

<img src="https://github.com/Lahorde/airbeam/blob/AirBeam_CairsensN02/img/air_casting_no2_screenshot.png?raw=true" width="500">

# References

* [Reprogram your AirBeam](http://www.takingspace.org/reprogram-your-airbeam/)
* [A post on taking space about this modification](http://www.takingspace.org/)
