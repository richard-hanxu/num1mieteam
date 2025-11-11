### Start
Follow the steps to install virtual ports for BLE here: https://github.com/Jakeler/ble-serial 

### Requirements
numpy, ble-serial
### bluetooth/bluetooth_test.py 
Program that will run on laptop, sends commands to Arduino #1 on robot to get sensor readings/send motor commands.
### bluetooth_relay_1/bluetooth_relay_1.ino 
Code for Arduino #1. Gets messages from Bluetooth and either returns sensor readings (which are connected to Arduino #1) or relays a motor command further down to Arduino #2.
### bluetooth_relay_2/bluetooth_relay_2.ino
Code for Arduino #2. Gets messages from Arduino #1 and performs the desired motor commands. Will return an encoder reading after it is finished driving.

### How To Run Particle Filter Simulation
- Make sure you are using the config.py file in this repo.
- Run simmer.py
- In another terminal, run mcl_simmer.py
- After any drive command (w0, r0, 'plan'), the robot will try to localize based on its surroundings and output its predicted position



