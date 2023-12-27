# Sensor fusion of IMU-Lidar-Radar data using Extended Kalman filter

<p>Converted the given "data_ekf.txt" text file to csv file manually. Organized the data by labelling them and segregating them into Lidar and Radar data. Grouped the Timestamp values of Radar and Lidar together in the same column. Assigned the value of "1" to Lidar readings and value of "2" to Radar readings. <br> </p>
<p>"1" and "2" will be used to reference Radar and Lidar in the code. Post converting the ".txt" file to ".csv" file, the contents of the file are read. csvread(filename, R1, C1, Range) : R1 = 1, C1 = 1 - I'm neglecting 0th row and 0th column of the csv file. My range is from B2 to L501. </p>

## x - State vector

<p> State of the car at any particular time instant could be denoted by the positions, velocities, yaw and yaw_rate. x is a 6*1 matrix. x is also referred to as the mean 
state vector because the position and velocities are represented by Gaussian distribution 
with mean x. </p>

<img width="250" alt="Screenshot 2023-12-26 222752" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/9293f9b0-c2f9-409f-8462-8aec462b9636">

<p> The states of the car are predicted by taking into consideration the time elapsed between two consecutive observations. <br>
We consider the time difference between two consecutive timestamp readings of a
particular sensor. Both the sensors have the same time elapsed which equals 0.1 milli
seconds. We then convert the milli seconds to seconds. </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM -1](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/e3a10a39-af6e-4199-9ac4-b2859087e6d5)

## A – State Transition Matrix (6*6 matrix)
<p>A can be computed using the motion model which provides us a way to update the
current state using old positions, displacement times and noise. <br> </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 3](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/4a8549e1-50ed-45de-a526-6985773d5111)

<p>Assuming Yaw and Yaw_rate are independent of X, Y, Vx and Vy. <br> </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/3fa85405-5af0-4c25-a71a-ecd3fa3448a4)

<p>The error terms in a Gaussian distribution are assumed to have zero mean and a
covariance of Q. </p>

## Q – Motion Noise 
<p>Uncertainty in the object’s position when predicting location. Q depends on the elapsed time and the uncertainty of acceleration. </p>
