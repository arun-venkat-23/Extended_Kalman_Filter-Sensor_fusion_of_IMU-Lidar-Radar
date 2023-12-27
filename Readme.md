# Sensor fusion of IMU-Lidar-Radar data using Extended Kalman filter

<p>Converted the given "data_ekf.txt" text file to csv file manually. Organized the data by labelling them and segregating them into Lidar and Radar data. Grouped the Timestamp values of Radar and Lidar together in the same column. Assigned the value of "1" to Lidar readings and value of "2" to Radar readings. <br> </p>
<p>"1" and "2" will be used to reference Radar and Lidar in the code. Post converting the ".txt" file to ".csv" file, the contents of the file are read. csvread(filename, R1, C1, Range) : R1 = 1, C1 = 1 - I'm neglecting 0th row and 0th column of the csv file. My range is from B2 to L501. </p>

## x - State vector

<p> State of the car at any particular time instant could be denoted by the positions, velocities, yaw and yaw_rate. x is a 6*1 matrix. x is also referred to as the mean 
state vector because the position and velocities are represented by Gaussian distribution 
with mean x. </p>

