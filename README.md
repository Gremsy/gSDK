Gimbal Interface Example
========================

This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Gremsy's Gimbal and an Onboard computer.

This example will receive one MAVLink message and send one MAVLink message.


Building
========

```
$ cd gSDK/
$ make
```

Hardware Setup
=========

Connect the COM2 cable to your Onboard Computer.  

The hardware connection between an GIMBAL T3 and a PC or Linux machine. Note that:

The recommended choice of USB to TTL cable is FT232 module

Execution
=========

You have to pick a port name, try searching for it with 
```

$ ls /dev/ttyACM* 
$ ls /dev/ttyUSB*
```

Run the example executable on the host shell:

```
$ cd gSDK/
$ ./gSDK -d /dev/ttyUSB0
```

To stop the program, use the key sequence `Ctrl-C`.

Here's an example output:
$ ./gSDK -d /dev/ttyUSB1 
OPEN PORT
Connected to /dev/ttyUSB1 with 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)

START READ THREAD 

Found

GOT GIMBAL SYSTEM ID: 20
START WRITE THREAD 

READ SOME MESSAGES 


Got message gimbal status 
Gimbal is running in FOLLOW mode!
Got message ATT 
    att  : time: 715054317 p:-0.072667 r:-0.127920 y:-0.623558 (degree)
Got message Mount orientation 
    orientation  :  time: 715084382  p:0.108494 r:-0.135546 y:0.087891 (degree)
Got message Mount status 
    mnt  :  time: 1566010215715390 p:5072 r:19408 y:16 (cnt)

READ SOME MESSAGES 

Got message gimbal status 
Gimbal is running in FOLLOW mode!
Got message ATT 
    att  : time: 715054317 p:-0.072667 r:-0.127920 y:-0.623558 (degree)
Got message Mount orientation 
    orientation  :  time: 715084382  p:0.108494 r:-0.135546 y:0.087891 (degree)
Got message Mount status 
    mnt  :  time: 1566010215715390 p:5072 r:19408 y:16 (cnt)

^C
TERMINATING AT USER REQUEST

CLOSE THREADS

CLOSE PORT
