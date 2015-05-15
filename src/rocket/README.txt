Any programs that need GPIO access needs the super user to run the program. 
To enter super user, use the command "sudo su".
To switch back to user pi, use the command "su pi".

To start the program, navigate to the folder "shell" and run "bash start.sh"
To stop the program, navigate to the folder "shell" and run "bash stop.sh"

File Descriptions:
- ADS1015.py: starts node for ADC convertor that samples the high G accel
- LSM9DS0.py: starts node for IMU. Only using 6DOF
- gps_node.py: start node for communicating with Adafruit Ultimate GPS
- commander.py: performs position estimation and determines apogee
- deployment.py: wrapper for sending a HIGH signal to the deployment board.
- save_data_node.py: logs the specified data streams.

TODO:
- LSM9DS0 seems to have an issue with writing control registers
  From an ID stand point, connections are okay
- ASD1015 crashed suddenly. Unexplained
- for position estimation, the AND the imu and gps apogee detection signals
  each signal is OR-ed with an error detection boolean.
  deploy = (imu_apogee OR imu_error AND (gps_apogee OR gps_error)
