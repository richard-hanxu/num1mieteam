### Start
Follow the steps to install virtual ports for BLE here: https://github.com/Jakeler/ble-serial 

### Requirements
numpy, ble-serial

### bluetooth_test.py 
Program that will run on laptop, sends commands to Arduino #1 on robot to get sensor readings/send motor commands.
### bluetooth_relay_1.ino 
Code for Arduino #1. Gets messages from Bluetooth and either returns sensor readings (which are connected to Arduino #1) or relays a motor command further down to Arduino #2.
### bluetooth_relay_2.ino
Code for Arduino #2. Gets messages from Arduino #1 and performs the desired motor commands. Will return an encoder reading after it is finished driving.
