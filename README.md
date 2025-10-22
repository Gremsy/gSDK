Gimbal Interface Example
========================

This is a simple MAVLink to UART interface example for Unix systems that can allow communication between Gremsy's Gimbal and an Onboard computer.

This will provide examples of controlling and communicating with Gremsy's Gimbal.

Documentation
=============
  * [Compilation](#compilation)
  * [Building](#building)
  * [Hardware Setup](#hardware-setup)
  * [Execution](#execution)
  * [Menu Structure](#menu-structure)
    * [1. Connection Initialization](#1-connection-initialization)
    * [2. Gimbal Parameters Retrieval](#2-gimbal-parameters-retrieval)
    * [3. Parameter Verification](#3-parameter-verification)
    * [4. Firmware and Message Settings](#4-firmware-and-message-settings)
    * [5. Gimbal Configuration Prompts](#5-gimbal-configuration-prompts)
    * [6. Gimbal Control Mode Selection](#6-gimbal-control-mode-selection)
      * [6.1 OFF Gimbal (Option 0)](#61-off-gimbal-option-0)
      * [6.2 ON Gimbal (Option 1)](#62-on-gimbal-option-1)
      * [6.3 Change Mount Mode (Option 2)](#63-change-mount-mode-option-2)
      * [6.4 Gimbal Information (Option 3)](#64-gimbal-information-option-3)
      * [6.5 Config Gimbal Follow Parameter (Option 4)](#65-config-gimbal-follow-parameter-option-4)
      * [6.6 Set Gimbal to LOCK Mode (Option 5)](#66-set-gimbal-to-lock-mode-option-5)
      * [6.7 Set Gimbal to Move Angle in LOCK Mode (Option 6)](#67-set-gimbal-to-move-angle-in-lock-mode-option-6)
      * [6.8 Set Gimbal to Move Rate in LOCK Mode (Option 7)](#68-set-gimbal-to-move-rate-in-lock-mode-option-7)
      * [6.9 Set Gimbal to FOLLOW Mode (Option 8)](#69-set-gimbal-to-follow-mode-option-8)
      * [6.10 Set Gimbal to Move Angle in FOLLOW Mode (Option 9)](#610-set-gimbal-to-move-angle-in-follow-mode-option-9)
      * [6.11 Set Gimbal to Move Rate in FOLLOW Mode (Option 10)](#611-set-gimbal-to-move-rate-in-follow-mode-option-10)
      * [6.12 Set Gimbal to MAPPING Mode (Option 11)](#612-set-gimbal-to-mapping-mode-option-11)
      * [6.13 Set Gimbal to Return Home (Option 12)](#613-set-gimbal-to-return-home-option-12)
      * [6.14 Set Gimbal Reboot (Option 13)](#614-set-gimbal-reboot-option-13)
      * [6.15 Upgrade Firmware (Option 14)](#615-upgrade-firmware-option-14)
      * [6.16 Monitoring Encoder - Attitude - IMU (Option 15)](#616-monitoring-encoder---attitude---imu-option-15)
    * [7. Termination and Cleanup](#7-termination-and-cleanup)
  * [Usage](#usage)
  * [Troubleshooting](#troubleshooting)
  * [Contact](#contact)




Compilation
=========
In order to compile you need the following packages:


- CMake version 3.5 or higher

- G++ compiler or Clang compiler

- C and C++ standard libraries

Building
========
```
$ cd gSDK/
$ git submodule update --init --recursive
$ mkdir build && cd build
$ cmake ..
$ make
```
Note: If you are using USB to control the gimbal, you should enable DTR/RTS control by using the following command:
```
$ cd gSDK/
$ git submodule update --init --recursive
$ mkdir build && cd build
$ cmake -DENABLE_DTR_RTS=1 ..
$ make
```
Hardware Setup
=========

Connect the COM2 cable to your Onboard Computer.  
The hardware connection between an GIMBAL and a PC or Linux machine. Note that:
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

GOT [STIFF_TILT] 20

GOT [STIFF_ROLL] 70

GOT [STIFF_PAN] 60

GOT [FILTER_OUT] 1

GOT [PWR_TILT] 20

GOT [PWR_ROLL] 70

GOT [PWR_PAN] 60

GOT [FLW_LPF_TILT] 20

GOT [FLW_LPF_PAN] 20

GOT [RC_TYPE] 15

GOT [GYRO_LPF] 4

GOT [RC_LIM_MIN_TILT] -45

GOT [RC_LIM_MAX_TILT] 45

GOT [RC_LIM_MIN_ROLL] -40

GOT [RC_LIM_MAX_ROLL] 40

GOT [RC_LPF_TILT] 80

GOT [RC_LPF_ROLL] 80

GOT [RC_LPF_PAN] 80

GOT [FLW_WD_TILT] 5

GOT [FLW_WD_PAN] 20

GOT [RC_SPD_TILT] 30

GOT [RC_SPD_ROLL] 30

GOT [RC_SPD_PAN] 30

GOT [RC_REVERSE_AXIS] 24

GOT [VERSION_Y] 7

GOT [VERSION_Z] 3

GOT [RC_LIM_MIN_PAN] -320

GOT [RC_LIM_MAX_PAN] 320

GOT [MAV_TS_ENCNT] 1

Check [VERSION_X] 7

Check [VERSION_Y] 7

Check [VERSION_Z] 3

Check [STIFF_TILT] 20

Check [STIFF_ROLL] 70

Check [STIFF_PAN] 60

Check [PWR_TILT] 20

Check [PWR_ROLL] 70

Check [PWR_PAN] 60

Check [FILTER_OUT] 1

Check [GYRO_LPF] 4

Check [FLW_LPF_TILT] 20

Check [FLW_LPF_PAN] 20

Check [FLW_WD_TILT] 5

Check [FLW_WD_PAN] 20

Check [RC_SPD_TILT] 30

Check [RC_SPD_ROLL] 30

Check [RC_SPD_PAN] 30

Check [RC_LPF_TILT] 80

Check [RC_LPF_ROLL] 80

Check [RC_LPF_PAN] 80

Check [RC_REVERSE_AXIS] 24

Check [MAV_TS_ENCNT] 1

Check [RC_LIM_MIN_TILT] -45

Check [RC_LIM_MAX_TILT] 45

Check [RC_LIM_MIN_ROLL] -40

Check [RC_LIM_MAX_ROLL] 40

Check [RC_LIM_MIN_PAN] -320

Check [RC_LIM_MAX_PAN] 320

Check [RC_TYPE] 15

Use mavlink gimbal V1
Gimbal Firmware version is 7.8.3.OFFICIAL
Set gimbal send raw encoder value.

GET GIMBAL DEVICE: 

Vendor name: GREMSY

Model name: MIO

Request gimbal device information.

Please Enter y/n(yes or no) to setting gimbal mavlink message rate

n
Please Enter y/n(yes or no) to setting gimbal stiffness - follow param

n
Please Enter number to select type of gimbal

         1. Two axis

         2. Three axis

2
Please Enter number to select three axis gimbal mode 

         1. Normal mode

         2. Inverted mode

1
You selected gimbal mount mode 
Three axis Normal Mount


 Please Enter number [0-15] to seclect Gimbal control mode
         0.   OFF Gimbal
         1.   ON Gimbal
         2.   Change mount mode
         3.   Gimbal information
         4.   Config gimbal follow parameter
         5.   Set Gimbal to LOCK mode
         6.   Set Gimbal to move angle in LOCK mode
         7.   Set Gimbal to move rate in LOCK mode
         8.   Set Gimbal to FOLLOW mode
         9.   Set Gimbal to move angle in FOLLOW mode
         10.  Set Gimbal to move rate in FOLLOW mode
         11.  Set Gimbal to MAPPING mode
         12.  Set Gimbal to Return Home
         13.  Set Gimbal Reboot
         14.  Upgrade Firmware
         15.  Monitoring Encoder - Attitude - IMU
^C
TERMINATING AT USER REQUEST

CLOSE THREADS

CLOSE PORT
```
# Menu Structure

## 1. Connection Initialization

- **OPEN PORT**: Initializes the connection to the specified port.
- **START WRITE THREAD**: Begins the thread responsible for sending data to the gimbal.
- **START READ THREAD**: Begins the thread responsible for receiving data from the gimbal.
- **Lost Connection!**: Indicates that the connection to the gimbal was lost. This message may repeat if reconnection attempts fail.
- **Found GIMBAL [SysID][CompID]: [1][154]**: Indicates successful detection of the gimbal with system ID 1 and component ID 154.

## 2. Gimbal Parameters Retrieval

- **GOT [PARAMETER] [VALUE]**: Lists the retrieved gimbal parameters and their respective values. These parameters include stiffness, power, filter settings, RC limits, and other configuration values.
  
  Example parameters:
  - `VERSION_X`, `VERSION_Y`, `VERSION_Z`
  - `STIFF_TILT`, `STIFF_ROLL`, `STIFF_PAN`
  - `PWR_TILT`, `PWR_ROLL`, `PWR_PAN`
  - `FLW_LPF_TILT`, `FLW_LPF_PAN`
  - `RC_TYPE`
  - `GYRO_LPF`
  - `RC_LIM_MIN_TILT`, `RC_LIM_MAX_TILT`, etc.
## 3. Parameter Verification

- **Check [PARAMETER] [VALUE]**: Confirms that the retrieved parameters match the expected values.

## 4. Firmware and Message Settings

- **Use mavlink gimbal V1**: Indicates the use of MAVLink protocol version 1 for communication.
- **Gimbal Firmware version is 7.7.3.OFFICIAL**: Displays the current firmware version of the gimbal.
- **Request gimbal device information**: Queries for the device's vendor and model information.
  - **Vendor name**: GREMSY
  - **Model name**: MIO

## 5. Gimbal Configuration Prompts
- **Please Enter y/n(yes or no) to setting gimbal mavlink message rate**: Prompts the user to confirm whether to set gimbal messages rate.
- **Please Enter y/n (yes or no) to setting gimbal stiffness - follow param**: Prompts the user to confirm whether to set gimbal stiffness and follow parameters.
- **Please Enter number to select type of gimbal**: Prompts the user to select the gimbal type.
  - Options:
    - `1. Two axis`
    - `2. Three axis`
- **Please Enter number to select two axis gimbal mount mode**: Prompts the user to select the mount mode for a two-axis gimbal.
  - If `1. Two axis` is selected, additional prompts will appear:
    - `1. Roll-Tilt Mount`
    - `2. Pan-Tilt Mount`
  - If `2. Three axis` is selected, additional prompts will appear:
    - `1. Normal mode`
    - `2. Inverted mode`
## 6. Gimbal Control Mode Selection

- **Please Enter number [0-15] to select Gimbal control mode**: Prompts the user to select the desired gimbal control mode.
  - Options:
    - `0.  OFF Gimbal`
    - `1.  ON Gimbal`
    - `2.  Change mount mode`
    - `3.  Gimbal information`
    - `4.  Config gimbal follow parameter`
    - `5.  Set Gimbal to LOCK mode`
    - `6.  Set Gimbal to move angle in LOCK mode`
    - `7.  Set Gimbal to move rate in LOCK mode`
    - `8.  Set Gimbal to FOLLOW mode`
    - `9.  Set Gimbal to move angle in FOLLOW mode`
    - `10. Set Gimbal to move rate in FOLLOW mode`
    - `11. Set Gimbal to MAPPING mode`
    - `12. Set Gimbal to Return Home`
    - `13. Set Gimbal Reboot`
    - `14. Upgrade Firmware`
    - `15. Monitoring Encoder - Attitude - IMU`
 
   ### 6.1 OFF Gimbal (Option 0)
   Turns off the gimbal.
   
   ### 6.2 ON Gimbal (Option 1)
   Turns on the gimbal.
   
   ### 6.3 Change Mount Mode (Option 2)
   Similar to **5. Gimbal Configuration Prompts**, this option allows the user to change the mount mode of the gimbal. The prompts for changing the mount mode are as follows:
   
   - **Please Enter number to select type of gimbal**: Prompts the user to select the gimbal type.
     - Options:
       - `1. Two axis`
       - `2. Three axis`
   - **Please Enter number to select two axis gimbal mount mode**: Prompts the user to select the mount mode for a two-axis gimbal.
     - If `1. Two axis` is selected, additional prompts will appear:
       - `1. Roll-Tilt Mount`
       - `2. Pan-Tilt Mount`
     - If `1. Three axis` is selected, additional prompts will appear:
       - `1. Normal mode`
       - `2. Inverted mode`
   
   ### 6.4 Gimbal Information (Option 3)
   Displays information about the gimbal.
   - **Gimbal Mount Mode**: Shows the current gimbal mount mode, such as Two axis Roll-Tilt mount, Two axis Pan-Tilt Mount, Three axis Inverted Mount, or Three axis Normal Mount.
   - **Gimbal Mode**: Displays the current gimbal operational mode.
   - **Gimbal State**: Shows the current state of the gimbal.
   - **Gimbal IMU Data**: Displays raw IMU data including accelerometer and gyroscope readings.
   - **Gimbal Attitude**: Shows the current pitch, roll, and yaw angles of the gimbal.
   - **Gimbal Encoder Data**: Displays the encoder values for pitch, roll, and yaw.
   - **Gimbal Configuration for Tilt, Roll, and Pan Axes**: Shows the current configuration settings for each axis, including direction, speed control, smooth control, smooth follow, and window follow parameters.
   - **Gimbal Limit Angles**: Displays the maximum and minimum angles for pitch, roll, and yaw.
   - **Motor Control Settings**: Shows the current motor control settings including hold strength, stiffness, gyro filter, and output filter.
  
   For example:
   ```
  Gimbal Firmware version is 7.7.3.OFFICIAL

  You selected gimbal mount mode 
  Two axis Roll-Tilt mount

  Gimbal mode: 2

  Gimbal state: 2

  Raw imu:  xacc:200, yacc:-218, zacc:8174, xgyro:8, xgyro:-8, xgyro:-36(raw)

  Gimbal attitude Pitch - Roll - Yaw: 0.00 - -0.00 - -0.01

  Gimbal encoder Pitch - Roll - Yaw: 0.00 - -0.00 - -0.01

  Config follow TILT: Dir: 0 -- Speed control: 30 -- Smooth control: 80 -- Smooth follow: 20 -- Window follow: 5

  Config follow ROLL: Dir: 0 -- Speed control: 30 -- Smooth control: 80 -- Smooth follow: 0 -- Window follow: 0

  Config follow PAN: Dir: 0 -- Speed control: 30 -- Smooth control: 80 -- Smooth follow: 20 -- Window follow: 20

  Pitch max: 45 -- Pitch min: -45

  Roll max: 40 -- Roll min: -40

  Yaw max: 320 -- Yaw min: -320

  Tilt hold strength: 20 -- Tilt stiffness: 20

  Roll hold strength: 70 -- Roll stiffness: 70

  Pan hold strength: 60 -- Pan stiffness: 60

  Gyro filter: 4

  Output filter: 1

   ```
   ### 6.5 Config Gimbal Follow Parameter (Option 4)
    Allows the user to configure the follow parameters for the gimbal.
    
    - **Chose axis**: Prompts the user to select the axis for which they want to configure the follow parameters.
      - Options:
        - `1: Tilt`
        - `2: Roll`
        - `3: Pan`
        - `4: Reset parameter`
        - `Other: Exit`

      Example Process:
      1. **Chose axis**: The user is prompted to choose an axis to configure.
         - Input: `1` (Tilt)
         - Output: "You have chosen 1"
      2. **Direction**: The user is prompted to set the direction (0 for Clockwise, 1 for Counter clockwise).
         - Input: `0`
      3. **Speed control**: The user sets the speed control parameter.
         - Input: `180`
      4. **Smooth control**: The user sets the smooth control parameter.
         - Input: `50`
      5. **Smooth follow**: The user sets the smooth follow parameter.
         - Input: `10`
      6. **Window follow**: The user sets the window follow parameter.
         - Input: `10`

  After inputting the parameters, the system will display the configured settings:
  - Output: `Config TILT: Dir: 0 -- Speed control: 180 -- Smooth control: 50 -- Smooth follow: 10 -- Window follow: 10`

  The user is then prompted to continue or not.
  - Input: `y` to continue or `n` to exit.

  If the user chooses to reset parameters:
  - Input: `4` (Reset parameter)
  - The parameters will be reset to their default values.
   
   ### 6.6 Set Gimbal to LOCK Mode (Option 5)
   Sets the gimbal to LOCK mode, where the gimbal maintains its current orientation regardless of the movement of the base.
   
   ### 6.7 Set Gimbal to Move Angle in LOCK Mode (Option 6)
   Allows the user to set a specific angle for the gimbal to move to while in LOCK mode.
  
   Sets the gimbal to LOCK mode and moves it to a specified angle.

   - **ROLL angle**: The desired angle for the roll axis.
   - **PITCH angle**: The desired angle for the pitch axis.
   - **YAW angle**: The desired angle for the yaw axis.
     
   **Note that this function utilizes the coordinate system [-180, 180].**
   ### 6.8 Set Gimbal to Move Rate in LOCK Mode (Option 7)
   In LOCK mode, this option allows the user to set the gimbal's movement rate.
   

   ### 6.9 Set Gimbal to FOLLOW Mode (Option 8)
   Sets the gimbal to FOLLOW mode, where the gimbal follows the movement of the attached device.
   
   ### 6.10 Set Gimbal to Move Angle in FOLLOW Mode (Option 9)
   In FOLLOW mode, this option allows the user to set the gimbal to a specific angle.
  
   Sets the gimbal to FOLLOW mode and moves it to a specified angle.

   - **ROLL angle**: The desired angle for the roll axis.
   - **PITCH angle**: The desired angle for the pitch axis.
   - **YAW angle**: The desired angle for the yaw axis.
     
   **Note that this function utilizes the coordinate system [-180, 180].**
   ### 6.11 Set Gimbal to Move Rate in FOLLOW Mode (Option 10)
   In FOLLOW mode, this option allows the user to set the gimbal's movement rate.
   
   ### 6.12 Set Gimbal to MAPPING Mode (Option 11)
   Sets the gimbal to MAPPING mode, which may be used for specific applications such as 3D mapping or scanning.
   
   ### 6.13 Set Gimbal to Return Home (Option 12)
   Sets the gimbal to return to its home position.
   
   ### 6.14 Set Gimbal Reboot (Option 13)

  Reboots the gimbal. Please note that it takes about 10 seconds to reboot successfully.

   ### 6.15 Upgrade Firmware (Option 14)

  Upgrade the gimbal firmware using a local hex file.

  Enter the path of the firmware file.

  Example:
  ```
  Enter a path: /home/Gremsy/User/gSDK/FW/gremsyMio-2Axis_v783_Official.hex
  The path: /home/Gremsy/User/gSDK/FW/gremsyMio-2Axis_v783_Official.hex
  CLOSE THREADS

  CLOSE PORT

  OPEN PORT
  Connected to /dev/ttyUSB0 with 460800 baud, 8 data bits, even parity, 1 stop bit (8N1)
  Entering bootloader mode...

  [✓] Parse Hex File
  [✓] Connecting
  [✓] Get CMD
  [✓] Get ID
  [✓] Erase memory
  [==================================================] 100% Flash memory                              
  Code flashed successfully!

  ```
  **Note**: If the upgrade is interrupted and gSDK cannot reconnect, you can use the upgrade_gimbal example for upgrading.
   ### 6.16 Monitoring Encoder - Attitude - IMU (Option 15)

   Monitoring Encoder - Attitude - IMU

   Example:
   ```
   Duration (in seconds): 
  2


  Raw imu:  xacc:234, yacc:-12, zacc:8034, xgyro:-32, xgyro:-21, xgyro:-35(raw)

  Gimbal attitude Pitch - Roll - Yaw: (0.40) - (-0.30) - (0.00)

  Gimbal encoder Pitch - Roll - Yaw: (-23336) - (-216) - (0)



  Raw imu:  xacc:-851, yacc:-15, zacc:8014, xgyro:-70, xgyro:1019, xgyro:-5(raw)

  Gimbal attitude Pitch - Roll - Yaw: (0.56) - (6.29) - (-0.00)

  Gimbal encoder Pitch - Roll - Yaw: (-23348) - (1024) - (0)



  Raw imu:  xacc:-2342, yacc:-20, zacc:7639, xgyro:-60, xgyro:323, xgyro:-37(raw)

  Gimbal attitude Pitch - Roll - Yaw: (0.28) - (18.23) - (-0.00)

  Gimbal encoder Pitch - Roll - Yaw: (-23276) - (3252) - (0)



  Raw imu:  xacc:-3069, yacc:-37, zacc:7401, xgyro:-34, xgyro:55, xgyro:-34(raw)

  Gimbal attitude Pitch - Roll - Yaw: (0.32) - (23.68) - (0.00)

  Gimbal encoder Pitch - Roll - Yaw: (-23268) - (4276) - (0)
   ```
## 7. Termination and Cleanup

- **TERMINATING AT USER REQUEST**: Indicates that the user has requested to terminate the program.
- **CLOSE THREADS**: Closes the read and write threads.
- **CLOSE PORT**: Closes the port connection.

# Usage

Follow the prompts to configure and control the gimbal. Input the required values as instructed by each prompt. This menu system is designed to facilitate easy interaction with the gimbal for setup and operation.

# Troubleshooting

- Ensure the correct port and baud rate are configured.
- If you see repeated "Lost Connection!" messages, check the physical connection and try reconnecting.
- Follow the prompts carefully to avoid configuration errors.

# Contact

For further assistance, please contact support at [support@gremsy.com].
