Gimbal Interface Example
========================

This is a simple MAVLink to UART interface example for Unix systems that can allow communication between Gremsy's Gimbal and an Onboard computer.

This example will receive one MAVLink message and send one MAVLink message.

What's new?
========

- Add function control gimbal follow fight controller " set_gimbal_follow_fc_sync ".
- Changed state running sdk (only control gimbal follow fc).
- Enter angle pitch, yaw from terminal.

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

Found GIMBAL [SysID][CompID]: [1][154]

GOT [VERSION_X] 7
GOT [PITCH_P] 30
GOT [PITCH_I] 120
GOT [VERSION_Y] 8
GOT [VERSION_Z] 131
GOT [ROLL_P] 150
GOT [YAW_P] 100
GOT [PITCH_POWER] 20
GOT [ROLL_POWER] 40
GOT [YAW_POWER] 45
GOT [YAW_I] 3
GOT [GYRO_LPF] 2
GOT [PITCH_FOLLOW] 100
GOT [YAW_FOLLOW] 100
GOT [PITCH_FILTER] 20
GOT [YAW_FILTER] 20
GOT [TILT_WINDOW] 2
GOT [PAN_WINDOW] 2
GOT [RC_PITCH_SPEED] 50
GOT [RC_ROLL_SPEED] 0
GOT [RC_YAW_SPEED] 50
GOT [RC_PITCH_LPF] 20
GOT [RC_ROLL_LPF] 20
GOT [RC_YAW_LPF] 20
GOT [JOY_AXIS] 24
GOT [ENC_TYPE_SEND] 1
GOT [TRAVEL_MIN_PIT] -90
GOT [TRAVEL_MAX_PIT] 90
GOT [TRAVEL_MIN_ROLL] -80
GOT [TRAVEL_MAX_ROLL] 80
GOT [TRAVEL_MIN_PAN] -180
GOT [TRAVEL_MAX_PAN] 180
GOT [RADIO_TYPE] 15
Check [VERSION_X] 7
Check [VERSION_Y] 8
Check [VERSION_Z] 131
Check [PITCH_P] 30
Check [ROLL_P] 150
Check [YAW_P] 100
Check [PITCH_POWER] 20
Check [ROLL_POWER] 40
Check [YAW_POWER] 45
Check [YAW_I] 3
Check [GYRO_LPF] 2
Check [PITCH_I] 120
Check [PITCH_FOLLOW] 100
Check [YAW_FOLLOW] 100
Check [PITCH_FILTER] 20
Check [YAW_FILTER] 20
Check [TILT_WINDOW] 2
Check [PAN_WINDOW] 2
Check [RC_PITCH_SPEED] 50
Check [RC_ROLL_SPEED] 0
Check [RC_YAW_SPEED] 50
Check [RC_PITCH_LPF] 20
Check [RC_ROLL_LPF] 20
Check [RC_YAW_LPF] 20
Check [JOY_AXIS] 24
Check [ENC_TYPE_SEND] 1
Check [TRAVEL_MIN_PIT] -90
Check [TRAVEL_MAX_PIT] 90
Check [TRAVEL_MIN_ROLL] -80
Check [TRAVEL_MAX_ROLL] 80
Check [TRAVEL_MIN_PAN] -180
Check [TRAVEL_MAX_PAN] 180
Check [RADIO_TYPE] 15
FW Version: 7.8.3.BETA
Set encoder messages rate: 10Hz
Set mount orientation messages rate: 100Hz
Set gimbal device attitude status messages rate: 10Hz
Set raw imu messgaes rate: 10Hz
Set gimbal send raw encoder value.
Request gimbal device information.
Set gimbal to LOCK MODE Successfully!
Enter yaw angle set point :

