# Multicopter-real-time-3D-simulation
A Matlab - Simulink based simulation tool for controlling multicopter in real-time with 3D visual 
http://thaibinhnguyen.xyz/project-view/he-thong-phan-mem-mo-phong-uav/

1. Overview
This project provide a collection of useful resources for beginners to understand multicopter control algorithm.
The project has come with real-time 3D visualization and a serial connection for RC input. All the visualization 
are based on Matlab plot, while controllers are designed in Simulink models.

2. Getting started
  2.1 Open and run (F5) "UART_Connector.m" in Matlab GUI.
  2.2 RC data are defined as pwm array of integer value which contains 4 elements for channels: roll, pitch, throttle, yaw.
  eg: 1500 1400 1300 1600
  2.3 Fundamental operation are followed video at my website.
  2.4 Multicopter aerodynamic model is in "dynamics.m"
  2.5 Controller models are in "multi_model*.slx" with * is 1,2,3..
3. Modify, conduct your own flight test and learn. 
