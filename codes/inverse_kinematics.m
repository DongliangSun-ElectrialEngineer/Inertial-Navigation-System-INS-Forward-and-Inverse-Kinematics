clear;close all;fclose all;clc;

%--> load data
load ('ems_simulated_trajectory.mat');

%--> Constants and Settings
we = 2*pi/(24*60*60);                   %Earth rotation rate (r/s)
a = 6378137;                            %Semi Major Axis of the Earth (in metres)
b = 6356752.3142;                       %Semi Minor Axis of the Earth (in metres)
e = sqrt((a^2-b^2)/a^2);                %Earth Shape Params
g_L = [0; 0; 9.8];                     %Gravity vector in the L frame
num_of_samples = length(ems_data.time);
sampling_period_sec = mean(diff(ems_data.time));
C_NL  = [0 1 0;1 0 0;0 0 -1];            %DCM from L to N frame
C_LN  = C_NL';                           %DCM from N to L frame
R2D = 180/pi;
D2R = pi/180;
Euler_roll_value     = ems_data.roll;
Euler_pitch_value    = ems_data.pitch;
Euler_azimuth_value  = ems_data.heading;
v_north_value = ems_data.vel(:,1);
v_east_value = ems_data.vel(:,2);
v_down_value = ems_data.vel(:,3);
vel = ems_data.vel; %Velocity in the L frame
alt_value  = ems_data.h;
lat_value  = ems_data.lat;
acc_IMU = ems_data.accel_clean;%IMU acceleration measurements
gyro_IMU = ems_data.gyro_clean;%IMU gyro measurements

%--> Initialize vectors sizes
acc  = zeros(num_of_samples-1,3);
gyro = zeros(num_of_samples-1,3);


for index = 1:num_of_samples-1
    %--> Calculate acceleration in the L frame
    acc(index,:) = (vel(index+1,:)-vel(index,:))/sampling_period_sec;
    
    %--> Calculate DCM 
    C_Lframe2body = angle2dcm(Euler_azimuth_value(index),Euler_pitch_value(index),Euler_roll_value(index),'ZYX');
    C_body2Lframe = C_Lframe2body'; % %DCM from the body frame to the L frame
    
    %--> Calculate Earth Carvature Rn and Rm
    lat_radian = lat_value(index);
    Rn = a/sqrt(1-(e^2)*(sin(lat_radian))^2);
    Rm = a*(1-e^2)/((1-(e^2)*(sin(lat_radian))^2)^(1.5));
    
    %--> Calculate transportation rate and earth rotation rate vectors in the L frame
    w_transportation_value = [ v_east_value(index)/(Rn + alt_value(index));
                              -v_north_value(index)/(Rm + alt_value(index));
                              -(v_east_value(index)*tan(lat_radian))/(Rn + alt_value(index))]; %Earth transportation rate as represented in L frame
    w_earth_value = [we*cos(lat_radian); 0; -we*sin(lat_radian)];  %Earth rotation rate as represented in L frame
    
    %--> Compensate acceleration for gravity and Coriolis effect
    acc_updated =  C_Lframe2body * (acc(index,:)' - g_L + cross((w_transportation_value + 2*w_earth_value),vel(index,:)'));
    acc(index,:) = acc_updated';
    
   
    C_Lframe2body_next = angle2dcm(Euler_azimuth_value(index+1),Euler_pitch_value(index+1),Euler_roll_value(index+1),'ZYX');
    C_body2Lframe_next = C_Lframe2body_next'; 
    
    %--> Calculate the skew matrix of the raw gyro measurements
    S_gyro = (C_Lframe2body*C_body2Lframe_next-eye(3))/sampling_period_sec;
    
    %--> Obtain the gyro vector
    gyro(index,:) = [S_gyro(3,2) S_gyro(1,3) S_gyro(2,1)];
    
    %--> Compensate gyro angle rate with the earth rate and transport rate
    gyro(index,:) = gyro(index,:) + (C_Lframe2body*(w_transportation_value + w_earth_value))';
end

%--> Plot your IMU data against the clean IMU data
NumRecords = num_of_samples-1;
time_vector_sec = ems_data.time(1:num_of_samples);

figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),acc_IMU(1:NumRecords,1),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),acc(1:NumRecords,1),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('forward acceleration (m/s^2)');xlabel('\textbf{time(s)}','Interpreter', 'latex');ylabel('$\mathbf{a_x (m/s^2)}$','Interpreter', 'latex')
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),acc_IMU(1:NumRecords,2),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),acc(1:NumRecords,2),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('right wing acceleration (m/s^2)');xlabel('\textbf{time(s)}','Interpreter', 'latex');ylabel('$\mathbf{a_y (m/s^2)}$','Interpreter', 'latex')
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),acc_IMU(1:NumRecords,3),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),acc(1:NumRecords,3),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('downward acceleration (m/s^2)');xlabel('\textbf{time(s)}','Interpreter', 'latex');ylabel('$\mathbf{a_z (m/s^2)}$','Interpreter', 'latex')


figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),gyro_IMU(1:NumRecords,1),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),gyro(1:NumRecords,1),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('roll rate (rad/s)');xlabel('time(s)');ylabel('\omega_x (rad/s)')
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),gyro_IMU(1:NumRecords,2),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),gyro(1:NumRecords,2),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('pitch rate (rad/s)');xlabel('time(s)');ylabel('\omega_y (rad/s)')
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),gyro_IMU(1:NumRecords,3),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),gyro(1:NumRecords,3),'r','LineWidth',1);grid on;
legend('Raw IMU data','Simulated IMU data');
title('azimuth rate (rad/s)');xlabel('time(s)');ylabel('\omega_z (rad/s)')