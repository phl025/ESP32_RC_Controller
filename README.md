# Arduino RC Controller for ESP32

## Description
This is a RC controller including engine sound player, PWM outputs for servos, ESC drivers and digital outputs.

## Details
It is based on *'Rc_Engine_Sound_ESP32'* : https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32.

If you need to use it for trucks, cars, or other vehicles, it is easier to use the original version.

This version is modified for my boats.

## Software
Many functions are converted to 'object-oriented programming', such as :
- Receiver : RxBase, RxPpm, RxCrsf, RxSbus, RxPwm
- RX Channels : RxChannel, chTrigger
- ESC : escDriver for RZ7886 or IBT2
- ...

## Hardware
You can use the original hardware, but a modified version is available. 

Modifications : 
- Add pins for diagnostics or futures extensions.
- CH5, CH6 rename to IN1, IN2 (could be use for PWM input CH5, CH6 or 'motor encoder')
- Add input IN3, pin 39
- Add CH7, pin 15 (for steering servo in case of 3x ECS)

## PCB

See "hardware/pictures/"

![hardware/pictures/ESP32_RC_Controler_005.jpg](hardware/pictures/ESP32_RC_Controler_005.jpg "This is a sample image.")

