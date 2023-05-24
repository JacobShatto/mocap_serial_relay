# Mocap Serial Relay 

## Authors
- Prince Kuevor (kuevpr@umich.edu)
- Matthew Romano (mmroma@umich.edu)

## 0) Supported Motion Capture Systems
- Optitrack (Motive 2.0.0)
- Qualisys (QTM)

## 1) Overview

This project supports using a Beaglebone (or any computer really) as a motion capture serial relay. The Beaglebone receives motion capture data over ethernet, does some minor processing, and then sends the data out in serial packets over XBees to receiving vehicles. 

Using XBees to wirelessly transmit real-time motion capture data has been much more reliable than WiFi. We typically have about a 40ms delay from vehicle motion to feedback data received onboard when using XBees. With WiFi at times the delay would be low but it was always extremely inconsistent and caused instability in controllers.



You can integrate the "receive_serial_opti" code into your embedded platform to receive the serial data. It will take a little bit of work but shouldn't be too hard.

## 2) Compiling

> make

## 3) Serial Testing Programs

You can use the "Serial Testing Programs" to make sure that the actual serial connection and XBees are working as they should. Use "send_serial" and "receive_serial" as a pair (one on the sending side, one on the receiving side). 

### a) send_serial
Usage: 

> ./sendSerial <Serial Port> <Baud Rate> <Message> 

<Serial Port> = /dev/ttyUSB0, etc...

<Baud Rate> = 9600, 115200, etc...

<Message> = h, a, 556a, robots, etc...

### b) receive_serial
Usage: 
> ./receiveSerial <Serial Port> <Baud Rate> 

 <Serial Port> = /dev/ttyUSB0, etc...

 <Baud Rate> = 9600,115200, etc...

## 4) Optitrack Serial Relay 

'opti_serial_relay' is the sending program and 'receive_serial_opti' is the receiving program. You can test them together as a pair (one on sender, one on receiver). Also, you can integrate in the 'receive_serial_opti' code into your own flight controllers and programs if desired.

The 'opti_serial_relay' and 'quali_serial_relay' programs also have the ability to test fake motion capture data so you don't need to be running the full motion capture systems to test. Look at the help menu to see the usage.

### a) opti_serial_relay
Usage: 
> ./opti_serial_relay [options] 

Use the help flag to see the latest options
e.g.
> ./opti_serial_relay -h


### b) receive_serial_opti 

Note: 'receive_serial_opti' will work with 'quali_serial_relay' as well since the serial packets are the same, the only difference is the interface with the motion capture system.

Usage: 
> ./receive_serial_opti <Serial Port> <Baud Rate> 

 <Serial Port> = /dev/ttyUSB0, etc...

 <Baud Rate> = 9600,115200, etc...

## 5) Qualisys Serial Relay
 
## a) quali_serial_relay
Usage: 
> ./quali_serial_relay [options] 

Use the help flag to see the latest options
e.g.
> ./quali_serial_relay -h
  



