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

![WhatsApp Image 2023-12-26 at 10 26 32 PM-2](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/8a439cc1-97d3-46bf-9800-a6a8c7aab6d0)

<p>Assuming Yaw and Yaw_rate are independent of X, Y, Vx and Vy. <br> </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/3fa85405-5af0-4c25-a71a-ecd3fa3448a4)

<p>The error terms in a Gaussian distribution are assumed to have zero mean and a
covariance of Q. </p>

## Q – Motion Noise 
<p>Uncertainty in the object’s position when predicting location. Q depends on the elapsed time and the uncertainty of acceleration. </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 3](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/826f2ed7-d010-4c6c-9f7c-37500e17a132)
![WhatsApp Image 2023-12-26 at 10 26 32 PM - 5](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/33c78648-e5bc-4587-8417-388658d65795)

The error vector can be represented in matrix form as <br>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 6](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/62a68db0-2320-4db0-8ccb-b981e53b6d6f)

Covariance (Q) = Expectation (v * v<sup>T</sup>) <br>
Q = G *Expectation {a * a<sup>T</sup>} * G <br>
Q<sub>v</sub> = Expectation {a * a<sup>T</sup>} <br>
> Please look at readme to check Q<sub>v</sub> matrix. <br>
Q = G * Q<sub>v</sub> * G <br>
Q will be a square symmetric matrix.

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 7](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/2581c3d6-ab1a-4df6-94c4-c2f8319a420d)

## B – Control input matrix

Unknown. The object’s acceleration cannot be measured certainly. <br>
B = [0]
## u – Control Vector
u = [0] 

## P – State Covariance Matrix (6*6 matrix) 
<p>The range accuracy is 1-5 cm for position. The final vertical and horizontal accuracies that are achieved in the data are of order of 5 to 15 cm and 15-50 cm. As the accuracies have low values, the measured position value will be close to the actual values. We get the position values directly from the sensor. So, we assume the covariance for X, Y and yaw to be 1. Yaw is calculated from X and Y. Whereas the velocities and rate of change of yaw are not directly obtained from the sensor measurements. And we don’t know the accuracy values for above quantities. Thus, we assume high covariance values for Vx, Vy and yaw_rate. As per reference from literature, any value between 1000 and 1500 works well for the covariance values. So, 1200 has been assigned the covariance values for the above quantities. </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 8](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/79a8a24d-67be-4250-a885-2ff1592d1c32)

## H – Observation Matrix. (2*6 matrix) – For Lidar

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 9](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/f6a71014-f7d4-493b-a8f0-9fb2b4f4b1fe)

<p>Lidar only measures an object’s position (x,y) but we have 6 state variables. To eliminate Vx, Vy, yaw and yaw_rate, we model H as a 2*6 matrix with 0 in all indices except the pivot elements, which have a value of 1 to produce the desired output matrix. </p>

## Sensor Noise
Let delta be sensor measurement noise. <br>
delta = N (0, R) – Gaussian noise with zero mean and covariance R

#### R – Uncertainty in sensor measurements.
delta comes from R (uncertainty in sensor measurements)

> R <sub> LIDAR </sub>

R = Expectation {delta * delta <sup>T</sup>} <br>
R = { s<sub>x<sup>2</sup></sub>  s<sub>xy</sub> ; s<sub>yx</sub>  s<sub>y<sup>2</sup></sub> } <br>
Since x and y are uncorrelated: <br>
R = { s<sub>x<sup>2</sup></sub>  0 ; 0  s<sub>y<sup>2</sup></sub> } <br>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 10](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/9b6d73de-eab4-464b-b7cd-8dff2e4a537b)

> R <sub> RADAR </sub>

R = Expectation {delta * delta <sup>T</sup>} <br>
R = { s<sub>r<sup>2</sup></sub>  s<sub>r,rho</sub>  s<sub>r,drho</sub> ; s<sub>phi,r</sub>  s<sub>phi<sup>2</sup></sub>  s<sub>phi,drho</sub> ; s<sub>r,drho</sub>  s<sub>phi,drho</sub>  s<sub>drho<sup>2</sup></sub> } <br>
r, rho and drho are uncorrelated: <br>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 11](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/8dbf301d-5271-4a39-ac31-ee1a1c04fe16)

## Measurement Model

<p> Lidar is a linear model-based sensor. We employ the Kalman Filter algorithm. Using the above defined matrices and state observation values from the csv file, we apply the KF algorithm and compute the Kalman gain, updated Lidar state measurements and the optimized EKF path.</p>

<p>Radar is a nonlinear model-based sensor. We employ Extended Kalman Filter algorithm. We
assume prediction model to remain linear whereas the measurement model is non-linear. </p>

![WhatsApp Image 2023-12-26 at 10 26 32 PM - 12](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/e61b334f-ced6-4248-941c-321218f59d1e)
![WhatsApp Image 2023-12-26 at 10 26 32 PM - 13](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/fa9a96cf-72e7-43f4-ac0d-72e49b8b1950)
![WhatsApp Image 2023-12-26 at 10 26 32 PM - 14](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/30e6fd6b-132c-4e4b-860c-b300825036f2)
![WhatsApp Image 2023-12-26 at 10 26 32 PM - 15](https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/9292ee5a-97a5-4865-9a37-41bdf635ba1f)

## Results

<p> Yaw and Yaw_rates are calculated from Radar data and are computed in the matlab code.
Whereas the Yaw and Yaw_rates from Lidar data are computed in Excel and imported to Matlab. </p>

<p> Kalman filter algorithm for Lidar data and Extended Kalman filter algorithm for Radar data are run and the output updated state matrix is stores in x_measured. The respective Lidar measurement and Radar measurement updated data are stored in L_measured and
R_measured matrices. The filter optimized path data is stored in the EKF matrix. </p>

<p> The appended L_measured values are in the form of X and Y. They are plotted directly
as such eliminating the requirement of coordinate conversion. </p>

<p> The appended R_measured values are in the form of rho, phi and drho. They are
converted to cartesian coordinates for plotting. </p>

<p> The respective plots of the EKF, L_measured, Radar_polartocartesian_Cart, Ground
truth are plotted and displayed as the graph. </p>

<img width="603" alt="Screenshot 2023-12-26 232106" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/7d33da00-1d4c-48a6-a6a6-90337fe80cba">
<img width="592" alt="Screenshot 2023-12-26 232151" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/8352b289-e839-4236-9f05-e2bd582c4008">
<img width="596" alt="Screenshot 2023-12-26 232214" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/4797ca51-a477-4899-9559-062e0bd630db">
<img width="592" alt="Screenshot 2023-12-26 232233" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/e9b888f2-7b2f-46e2-b3cc-d83ec67bb760">
<img width="586" alt="Screenshot 2023-12-26 232253" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/f9f602f8-1b58-4920-a79f-b492256308ea">

The output of EKF is also attached to the repository. Please refer to the below table with regards to the variables in the output.csv file.

<img width="645" alt="Screenshot 2023-12-26 232318" src="https://github.com/arun-venkat-23/Extended_Kalman_Filter-Sensor_fusion_of_IMU-Lidar-Radar/assets/137104589/381d7635-baee-4b57-99d8-8092ab0fc762">
