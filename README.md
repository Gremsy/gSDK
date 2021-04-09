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

START READ THREAD 

 Lost Connection!
 Lost Connection!
Found 

GOT GIMBAL SYSTEM ID: 4
START WRITE THREAD 
GIMBAL_STATE_NOT_PRESENT
Request param read: VERSION_X 
Request param read: VERSION_Y 
Request param read: VERSION_Z 
GOT [VERSION_X] 7
GOT [VERSION_Y] 5
GOT [VERSION_Z] 0
Request param read: PITCH_P 
Request param read: ROLL_P 
Request param read: YAW_P 
GOT [ROLL_P] 90
Request param read: PITCH_POWER
GOT [PITCH_POWER] 40
Request param read: ROLL_POWER
GOT [ROLL_POWER] 40
Request param read: YAW_POWER
GOT [YAW_POWER] 40
Request param read: YAW_I
GOT [YAW_I] 3
Request param read: GYRO_LPF 
Request param read: PITCH_I 
GOT [GYRO_LPF] 2
Request param read: PITCH_FOLLOW 
Request param read: YAW_FOLLOW 
GOT [PITCH_FOLLOW] 65
Request param read: PITCH_FILTER 
Request param read: YAW_FILTER 
GOT [YAW_FOLLOW] 87
GOT [PITCH_FILTER] 50
Request param read: TILT_WINDOW 
Request param read: PAN_WINDOW 
GOT [YAW_FILTER] 50
GOT [TILT_WINDOW] 0
Request param read: RC_PITCH_SPEED 
GOT [PAN_WINDOW] 0
Request param read: RC_ROLL_SPEED 
Request param read: RC_YAW_SPEED 
GOT [RC_PITCH_SPEED] 50
GOT [RC_ROLL_SPEED] 50
Request param read: RC_PITCH_LPF 
GOT [RC_YAW_SPEED] 50
Request param read: RC_ROLL_LPF 
GOT [RC_PITCH_LPF] 50
Request param read: RC_YAW_LPF 
Request param read: JOY_AXIS 
GOT [RC_YAW_LPF] 70
Request param read: HEARTBEAT_EMIT 
Request param read: STATUS_RATE 
GOT [JOY_AXIS] 1
GOT [HEARTBEAT_EMIT] 1
Request param read: ENC_CNT_RATE 
Request param read: ENC_TYPE_SEND 
GOT [STATUS_RATE] 10
Request param read: ORIEN_RATE 
GOT [ENC_TYPE_SEND] 0
Request param read: IMU_RATE 
GOT [ORIEN_RATE] 50
GOT [IMU_RATE] 10
Request param read: PITCH_P 
Request param read: YAW_P 
Request param read: PITCH_I 
GOT [PITCH_P] 80
GOT [YAW_P] 100
Request param read: RC_ROLL_LPF 
Request param read: ENC_CNT_RATE 
GOT [PITCH_I] 120
GOT [ENC_CNT_RATE] 10
Request param read: RC_ROLL_LPF 
GOT [RC_ROLL_LPF] 60
Check [VERSION_X] 7 
Check [VERSION_Y] 5 
Check [VERSION_Z] 0 
Check [PITCH_P] 80 
Check [ROLL_P] 90 
Check [YAW_P] 100 
Check [PITCH_POWER] 40 
Check [ROLL_POWER] 40 
Check [YAW_POWER] 40 
Check [YAW_I] 3 
Check [GYRO_LPF] 2 
Check [PITCH_I] 120 
Check [PITCH_FOLLOW] 65 
Check [YAW_FOLLOW] 87 
Check [PITCH_FILTER] 50 
Check [YAW_FILTER] 50 
Check [TILT_WINDOW] 0 
Check [PAN_WINDOW] 0 
Check [RC_PITCH_SPEED] 50 
Check [RC_ROLL_SPEED] 50 
Check [RC_YAW_SPEED] 50 
Check [RC_PITCH_LPF] 50 
Check [RC_ROLL_LPF] 60 
Check [RC_YAW_LPF] 70 
Check [JOY_AXIS] 1 
Check [HEARTBEAT_EMIT] 1 
Check [STATUS_RATE] 10 
Check [ENC_CNT_RATE] 10 
Check [ENC_TYPE_SEND] 0 
Check [ORIEN_RATE] 50 
Check [IMU_RATE] 10 
GIMBAL_STATE_PRESENT_RUNNING 
READ SOME MESSAGES 

Got message gimbal status 
Gimbal is operating
Got message RAW IMU.
	raw imu: time: 1591149964954814, xacc:20, yacc:-60, zacc:8534, xgyro:1287, xgyro:110, xgyro:124(raw)
Got message Mount orientation.
	orientation: time: 2020804268, p:-0.002313, r:0.058161, y:-0.032959 (degree)
Got message Mount status 
	Encoder Angle: time: 1591149964954833, p:0, r:0, y:0 (Degree)
	SETTING TILT: dir 1, speed_follow: 65, speed_control: 50
	MOTOR_CONTROL: GYRO: 2, OUT 3, GAIN 120
	TILT  stiff 80, hold: 40
	ROLL  stiff 90, hold: 40
	PAN   stiff 100, hold: 40

FW Version: 7.5.0.OFFICIAL
READ SOME MESSAGES 

Got message gimbal status 
Gimbal is operating
Got message RAW IMU.
	raw imu: time: 1591149965057013, xacc:103, yacc:-168, zacc:8289, xgyro:502, xgyro:-43, xgyro:-14(raw)
Got message Mount orientation.
	orientation: time: 2020874504, p:-0.049958, r:0.079474, y:-0.032959 (degree)
Got message Mount status 
	Encoder Angle: time: 1591149965057037, p:0, r:0, y:0 (Degree)
	SETTING TILT: dir 1, speed_follow: 65, speed_control: 50
	MOTOR_CONTROL: GYRO: 2, OUT 3, GAIN 120
	TILT  stiff 80, hold: 40
	ROLL  stiff 90, hold: 40
	PAN   stiff 100, hold: 40

^C
TERMINATING AT USER REQUEST

CLOSE THREADS

CLOSE PORT
```
