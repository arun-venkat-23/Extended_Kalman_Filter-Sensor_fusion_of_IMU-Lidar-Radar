Yaw_Lidar = table2array(LidarYawData);

dat = csvread('EKF_Data_Main.csv',1,1);

%Initializing the variables
del_t = 0.1;                         % Taking the difference between two consecutive timesteps and then divide it by 1000000 to convert it from milliseconds to seconds           
R_measured = [];                     % Storing measured Radar data
L_measured = [];                     % Storing measured Lidar data        
EKF = [];                            % Storing the EKF data
x_measured = [];                     % Storing the updated x (state) values of Lidar and Radar
result = [];                         % To store the required output 
Yaw_lidar = [];

% STATE TRANSITION MATRIX
A = [[1, 0, del_t, 0, 0, 0];               
     [0, 1, 0, del_t, 0, 0];         
     [0, 0, 1, 0, 0, 0];             
     [0, 0, 0, 1, 0, 0];             
     [0, 0, 0, 0, 1, del_t];         
     [0, 0, 0, 0, 0, 1]];                        

% CONTROL INPUT MATRIX
B = 0;
% CONTROL VECTOR
u = 0;
        
%{ 
NOISE COVARIANCE MATRIX
Since our given model is discrete noise model as per the definition in
kalmanfilter.net, the values of sigma_square_ax, sigma_square_ay and
sigma_square_alpha must be less. Assuming it to be 1.
%}

sig_square_ax = 1;                         
sig_square_ay = 1;
sig_square_alphax = 1;
dt_pow_2 = del_t*del_t;
dt_pow_3 = del_t*del_t*del_t;
dt_pow_4 = del_t*del_t*del_t*del_t;

q11 = dt_pow_4*sig_square_ax/4;
q13 = dt_pow_3*sig_square_ax/2;
q22 = dt_pow_4*sig_square_ay/4;
q24 = dt_pow_3*sig_square_ay/2;
q31 = dt_pow_3*sig_square_ax/2;
q33 = dt_pow_2*sig_square_ax;
q42 = dt_pow_3*sig_square_ay/2;
q44 = dt_pow_2*sig_square_ay;
q55 = dt_pow_4*sig_square_alphax/4;
q56 = dt_pow_3*sig_square_alphax/2;
q65 = dt_pow_3*sig_square_alphax/2;
q66 = dt_pow_2*sig_square_alphax;

% NOISE COVARIANCE MATRIX
Q = [[q11 0 q13 0 0 0];
     [0 q22 0 q24 0 0];
     [q31 0 q33 0 0 0];
     [0 q42 0 q44 0 0];
     [0 0 0 0 q55 q56]
     [0 0 0 0 q65 q66]]; 

% STATE COVRIANCE MATRIX
 P = [[1, 0, 0, 0, 0, 0];
     [0, 1, 0, 0, 0, 0];
     [0, 0, 1200, 0, 0, 0];
     [0, 0, 0, 1200, 0, 0]
     [0, 0, 0, 0, 1, 0]
     [0, 0, 0, 0, 0, 1200]];

% LIDAR OBSERVATION MATRIX - LINEAR MODEL
H_L = [[1, 0, 0, 0, 0, 0];
     [0, 1, 0, 0, 0, 0]];

% LIDAR MEASUREMENT NOISE COVARIANCE MATRIX
R_L = [[0.02273259, 0];
       [0, 0.021209854]];

% RADAR MEASUREMENT NOISE COVARIANCE MATRIX
R_R = [[0.092799661, 0, 0];
      [0, 5.546829959, 0];
      [0, 0, 0.083025775]];

I = eye(6);

for k = 1:length(dat)
    % TO CHECK IF THE PARTICULAR SET OF DATA BELONGS TO RADAR OR LIDAR
    if (dat(k,1) == 1)
        x = [dat(k,2); dat(k,3); 0; 0; 0; 0];
    else
        x = [dat(k,2); dat(k,3); dat(k,4); 0; 0; 0];
    end
    
    if (dat(k,1) == 1)

        % KALMAN FILTER ALGORITHM FOR LIDAR DATA. LIDAR DATA RESEMBLES LINEAR MODEL     
        x = A * x;                                             % Prediction 
        P = A * P * transpose(A) + Q;                          % Priori Covariance
        Z = dat(k,2:3);
        y = transpose(Z) - (H_L * x);
        S = H_L * P * transpose(H_L) + R_L;
        Kalman_gain_L = P * transpose(H_L) * inv(S);           % Kalman Gain computation
        x = x + (Kalman_gain_L * y);                           % Updated measurement
        P = (I - (Kalman_gain_L * H_L)) * P;                   % Updated Posteriori Covariance

        % UPDATING EKF MATRIX WITH ALL THE UPDATED MEASUREMENT VALUES
        EKF = [EKF;[x(1),x(2)]];
        % UPDATING Lidar MEASUREMENT MATRIX - TO PLOT LIDAR DATA GRAPH
        L_measured = [L_measured; dat(k,2:3)];
      
       
    else
        % EXTENDED KALMAN FILTER ALGORITHM FOR RADAR DATA. RADAR DATA RESEMBLES NON LINEAR MODEL  
        x = A * x;                                         % Prediction
        P = A * P * transpose(A) + Q;                      % Priori Covariance

        % Polar coordinates to cartesian coordinates conversion 
        Z = dat(k,2:4);
        r = Z(1);
        theta = Z(2);
        rdot = Z(3);
        X = r*cos(theta);
        Y = r*sin(theta);
        VX = rdot*cos(theta);
        VY = rdot*sin(theta);

        % YAW AND YAW_RATE CALCULATION
        if(k==251)
            rt = [0, 0, 0];

        else 
            rt = [dat(k-1, 2:4)];
        end

        Yaw = atan((Y-rt(1)*sin(rt(2)))/(X-(rt(1)*cos(rt(2)))));
        Yaw_Rate = atan((rt(3)*sin(rt(2))-VY)/((rt(3)*cos(rt(2)))-VX));

        a = X^2 + Y^2;
        b = sqrt(a);
        c = a * b;

        %{
        Computing the Jacobian matrix post finding the relationship
        between the state variables and the measurement output variables
        of Radar
        %}

        h11 = X/b;
        h12 = Y/b;
        h21 = -Y/a;
        h22 = X/a;
        h31 = (Y*(VX*Y-VY*X))/c;
        h32 = (X*(X*VY-Y*VX))/c;
        h33 = X/b;
        h34 = Y/b;

        % JACOBIAN MATRIX - OBSERVTION MATRIX FOR NON LINEAR MODEL
        H_Jacobian = [[h11, h12, 0, 0, 0, 0];
                      [h21, h22, 0, 0, 0, 0];
                      [h31, h32, h33, h34, 0, 0]];
    
        Z_state = [X; Y; VX; VY; Yaw; Yaw_Rate];
        y = transpose(Z) - (H_Jacobian * Z_state);
        S = H_Jacobian * P * transpose(H_Jacobian) + R_R; 
        Kalman_gain_R = P * transpose(H_Jacobian) * inv(S);      % Kalman Gain 
        x = Z_state + (Kalman_gain_R * y);                       % Updated Measurement
        P = (I - (Kalman_gain_R * H_Jacobian)) * P;              % Updated Posteriori Covariance

        % UPDATING EKF MATRIX WITH ALL THE UPDATED MEASUREMENT VALUES
        EKF = [EKF;[x(1),x(2)]];
        % UPDATING Radar MEASUREMENT MATRIX - TO PLOT RADAR DATA GRAPH (DATA in rho, phi and drho format)
        R_measured = [R_measured; dat(k,2:4)];


    end
    % UPDATING STATE MATRIX WITH ALL THE UPDATED MEASUREMENT VALUES FROM BOTH LIDAR AND RADAR
    x_measured = [x_measured;transpose(x)];
end

for n= 1:250
    x_measured(n,3:6) = Yaw_Lidar(n,1:4);
end
%{
Converting the appended updated Radar measurement values (rho, phi and
drho) to cartesian coordinates (X,Y) for plotting
%}
for i = 1:length(R_measured)
    Radar_polartocartesian_Cart(i,:) = [[R_measured(i,1),0];[0, R_measured(i,1)]]*[cos(R_measured(i,2));sin(R_measured(i,2))];
end

% PLOTTING THE OUTPUT CURVES
hold on;
plot(dat(:,6),dat(:,7),'linewidth', 2);
scatter(EKF(:,1),EKF(:,2),25,'filled','r');
scatter(L_measured(:,1),L_measured(:,2),5,'filled','blue');
scatter(Radar_polartocartesian_Cart(:,1),Radar_polartocartesian_Cart(:,2),5,'filled','g');
legend('Groundtruth','EKF Path result','Lidar Measurement','Radar Measurement','Location','northwest');
axis square;
hold off;

% LOOP TO DISPLAY THE REQUIRED OUTPUT DATA
for n=1:length(x_measured)
    result(n,1) = dat(n,5);
    result(n,2:7) = x_measured(n,1:6);
    if(dat(n,1)==1)
        result(n,8) = 1;
        %result(n,9:10) = L_measured(n,1:2);
    else
        result(n,8) = 2;
        %result(n,9:10) = Radar_polartocartesian_Cart(n,1:2);
    end
    result(n,11:14) = dat(n,6:9);
end
for n = 1:250
    result(n,9:10) = L_measured(n,1:2);
end
for n = 251:500
    result(n,9:10) = Radar_polartocartesian_Cart(n-250,1:2);
end
OP = array2table(result,'VariableNames',{'Time','X_state','Y_state','VX_state','VY_state','Yaw_Angle_state','Yaw_Rate_state','Sensor_type','X_measured','Y_measured','X_groundtruth','Y_groundtruth','VX_groundtruth','VY_groundtruth'})
filename = ("Matlab_Output.xlsx");
writetable(OP,filename);
