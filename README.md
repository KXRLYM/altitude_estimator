## Altitude_estimator
A simple c++ implementation of sensor fusion for state estimation.

### Data
Altitude data was obtained using ArduPilot Terrain Generator: https://terrain.ardupilot.org/ around Mt Cootha, Brisbane. Then two sensor signals were made with added noises and running average.
![data](https://github.com/user-attachments/assets/bc20b89e-0dd5-44de-8690-063e88001c19)

### Output
Using the two ""timeseries"" data, altitude at each timestep is estimated. The output is dependent on the ratio of standard deviations of sensor noises as well as the process noise. 
For simplicity, a constant velocity model + same sensor update frequencies were used.
The following is an example with a higher uncertainty on the proximity sensor.
![high_gps](https://github.com/user-attachments/assets/4689b0ee-6244-4a5a-ab01-a7b2741a813d)



### Dependencies
- **cnpy** : converting npy arrays into Eigen
- **Eigen** : library to compute Kalman filter equations
- **matplotcplusplus** : visualisation
