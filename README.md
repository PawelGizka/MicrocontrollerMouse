Program to control a computer mouse by tilting a stm32 microcontroller. 
The microcontroller must by equiped with an accelerometer. 
Readout of the accelerometer indications is via I2C BUS. 
The microcontroller sends indications to computer via USART.

To Compile:

```
make
``` 

Note that you need to have ARM environment configured first.

Run:

```
./mouse.sh < /dev/ttyACM0
``` 