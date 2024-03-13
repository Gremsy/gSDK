Gimbal Interface Example
========================

This is a simple MAVLink to UART interface example for Unix systems that can allow communication between Gremsy's Gimbal and an Onboard computer.

This example will receive one MAVLink message and send one MAVLink message.


Building
========

```
$ cd gSDK/
$ mkdir build && cd build
$ cmake ..
$ make
```

Hardware Setup
=========

Connect the COM2 cable to your Onboard Computer.  
The hardware connection between an GIMBAL T3 and a PC or Linux machine. Note that:
The recommended choice of USB to TTL cable is FT232 module.

Refer to: gSDK Documentation

Execution
=========

An FTDI cable will show up on a `ttyUSB*`.
Run the example executable on the host shell:

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

```

$ ./gSDK -d /dev/ttyUSB0 
OPEN PORT
Connected to /dev/ttyUSB0 with 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)

START WRITE THREAD

 Lost Connection!

 Lost Connection!

START READ THREAD

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

 Lost Connection!

Found GIMBAL [SysID][CompID]: [1][154]

GOT [VERSION_X] 7

GOT [STIFF_TILT] 70

GOT [STIFF_ROLL] 80

GOT [STIFF_PAN] 90

GOT [FILTER_OUT] 1

GOT [PWR_TILT] 40

GOT [PWR_ROLL] 40

GOT [PWR_PAN] 40

GOT [FLW_LPF_TILT] 10

GOT [FLW_LPF_PAN] 10

GOT [RC_TYPE] 14

GOT [GYRO_LPF] 5

GOT [RC_LIM_MIN_TILT] -45

GOT [RC_LIM_MAX_TILT] 90

GOT [RC_LIM_MIN_ROLL] -45

GOT [RC_LIM_MAX_ROLL] 45

GOT [RC_LPF_TILT] 50

GOT [RC_LPF_ROLL] 50

GOT [RC_LPF_PAN] 60

GOT [FLW_WD_TILT] 0

GOT [FLW_WD_PAN] 0

GOT [RC_SPD_TILT] 60

GOT [RC_SPD_ROLL] 60

GOT [RC_SPD_PAN] 60

GOT [RC_REVERSE_AXIS] 8

GOT [VERSION_Y] 8

GOT [VERSION_Z] 194

GOT [RC_LIM_MIN_PAN] -70

GOT [RC_LIM_MAX_PAN] 70

GOT [MAV_TS_ENCNT] 1

Check [VERSION_X] 7

Check [VERSION_Y] 8

Check [VERSION_Z] 194

Check [STIFF_TILT] 70

Check [STIFF_ROLL] 80

Check [STIFF_PAN] 90

Check [PWR_TILT] 40

Check [PWR_ROLL] 40

Check [PWR_PAN] 40

Check [FILTER_OUT] 1

Check [GYRO_LPF] 5

Check [FLW_LPF_TILT] 10

Check [FLW_LPF_PAN] 10

Check [FLW_WD_TILT] 0

Check [FLW_WD_PAN] 0

Check [RC_SPD_TILT] 60

Check [RC_SPD_ROLL] 60

Check [RC_SPD_PAN] 60

Check [RC_LPF_TILT] 50

Check [RC_LPF_ROLL] 50

Check [RC_LPF_PAN] 60

Check [RC_REVERSE_AXIS] 8

Check [MAV_TS_ENCNT] 1

Check [RC_LIM_MIN_TILT] -45

Check [RC_LIM_MAX_TILT] 90

Check [RC_LIM_MIN_ROLL] -45

Check [RC_LIM_MAX_ROLL] 45

Check [RC_LIM_MIN_PAN] -70

Check [RC_LIM_MAX_PAN] 70

Check [RC_TYPE] 14

Use mavlink gimbal V2
Gimbal Firmware version is 7.8.2.PREVIEW
Setting gimbal mavlink message
Set encoder messages rate: 10Hz

Set mount orientation messages rate: 10Hz

Set gimbal device attitude status messages rate: 10Hz

Set raw imu messgaes rate: 10Hz

Set gimbal send raw encoder value.

Request gimbal device information.

GET GIMBAL DEVICE:

Vendor name: GREMSY

Model name: T3V3 2Axis


 Please Enter y/n(yes or no) to setting gimbal stiffness - follow param
n

 Please Enter number to select two axis gimbal mount mode
         0. Roll-Tilt Mount
         1. Pan-Tilt Mount
1
You selected gimbal mode Pan-Tilt Mount

 Please Enter number [0-10] to seclect Gimbal control mode
         0. OFF Gimbal
         1. ON Gimbal
         2. Set Gimbal to LOCK mode
         3. Set Gimbal to move angle in LOCK mode
         4. Set Gimbal to move rate in LOCK mode
         5. Set Gimbal to FOLLOW mode
         6. Set Gimbal to move angle in FOLLOW mode
         7. Set Gimbal to move rate in FOLLOW mode
         8. Set Gimbal to MAPPING mode
         9. Set Gimbal to Return Home
         10. Set Gimbal Reboot

^C
TERMINATING AT USER REQUEST

CLOSE THREADS

CLOSE PORT
```
