clear;close all;fclose all;clc;

%--> load data
load ('ems_simulated_trajectory.mat');

%--> Constants and Settings
we = 2*pi/(24*60*60);                   %Earth rotation rate (r/s)
%we = 0;                                 %Teat the effect of neglecting the Earth's rotation rate
a = 6378137;                            %Semi Major Axis of the Earth (in metres)
b = 6356752.3142;                       %Semi Minor Axis of the Earth (in metres)
e = sqrt((a^2-b^2)/a^2);                %Earth Shape Params
g_N = [0; 0; -9.8];                     %Gravity vector in the N frame
num_of_samples = length(ems_data.time);
sampling_period_sec = mean(diff(ems_data.time));
C_NL  = [0 1 0;1 0 0;0 0 -1];            %DCM from L to N frame
C_LN  = C_NL';                           %DCM from N to L frame
R2D = 180/pi;
D2R = pi/180;
lat0 = ems_data.lat(1)*R2D;
lon0 = ems_data.lon(1)*R2D;
alt0 = ems_data.h(1);
use_clean_imu = 1; % Test the effect of IMU errors/noise on the accuracy of the output PVA

%--> Initialize vectors sizes
lat_value                               = zeros(num_of_samples,1);
lon_value                               = zeros(num_of_samples,1);
alt_value                               = zeros(num_of_samples,1);
v_north_value                           = zeros(num_of_samples,1);
v_east_value                            = zeros(num_of_samples,1);
v_up_value                              = zeros(num_of_samples,1);
Euler_pitch_value                       = zeros(num_of_samples,1);
Euler_roll_value                        = zeros(num_of_samples,1);
Euler_azimuth_value                     = zeros(num_of_samples,1);
quat_vector                             = zeros(num_of_samples,4);
P_east                                  = zeros(num_of_samples,1);
P_north                                 = zeros(num_of_samples,1);

%--> Initialize the state
Euler_roll_value(1)     = ems_data.roll(1);
Euler_pitch_value(1)    = ems_data.pitch(1);
Euler_azimuth_value(1)  = ems_data.heading(1);
quat_vector(1,:) = angle2quat(Euler_azimuth_value(1),Euler_pitch_value(1),Euler_roll_value(1),'ZYX');
lat_value(1)  = ems_data.lat(1)*R2D;
lon_value(1)  = ems_data.lon(1)*R2D;
alt_value(1)  = ems_data.h(1);
v_east_value(1) = ems_data.vel_N(1,1);
v_north_value(1) = ems_data.vel_N(1,2);
v_up_value(1) = ems_data.vel_N(1,3);
P_east(1) =  ems_data.east(1);
P_north(1) = ems_data.north(1);


%--> Add a random noise and bias to the z-axis of the gyroscope data
gyro_rw_stdv = 0.1; %deg/s
gyro_bias = 0.25; %deg/s
ems_data.gyro_noisy = ems_data.gyro_clean;
ems_data.accel_noisy = ems_data.accel_clean;
ems_data.gyro_noisy(:,3) = ems_data.gyro_clean(:,3) + ...
                           gyro_bias*D2R + ...
                           gyro_rw_stdv*D2R*randn(num_of_samples,1);

%--> Choose whether to use clean IMU data or noisy IMU data
if use_clean_imu == 1
    gyro = ems_data.gyro_clean;
    acc = ems_data.accel_clean;
else
    gyro = ems_data.gyro_noisy;
    acc = ems_data.accel_noisy;
end



for index = 1:num_of_samples-1
 
    %--> Convert the roll, pitch, and Azimuth into a Direction Cosine Matrix (DCM)
    C_Lframe2body = angle2dcm(Euler_azimuth_value(index),Euler_pitch_value(index),Euler_roll_value(index),'ZYX'); % convert vectors from the L frame to body frame
    C_body2Lframe = C_Lframe2body'; % %DCM from the body frame to the L frame
    
    lat_radian = lat_value(index)*D2R;
    %--> Calculate Earth Carvature Rn and Rm
    Rn = a/sqrt(1-(e^2)*(sin(lat_radian))^2);
    Rm = a*(1-e^2)/((1-(e^2)*(sin(lat_radian))^2)^(1.5));
    
    %--> Calculate transportation rate and earth rotation rate vectors in the N frame
    w_transportation_value = [-v_north_value(index)/(Rm + alt_value(index));
                              v_east_value(index)/(Rn + alt_value(index));
                             (v_east_value(index)*tan(lat_radian))/(Rn + alt_value(index))]; %Earth transportation rate as represented in N frame
    w_earth_value = [0; we*cos(lat_radian); we*sin(lat_radian)];  %Earth rotation rate as represented in N frame
    w_L_IL_value = C_LN*(w_transportation_value + w_earth_value);

    %--> Compensated gyro data
    gyro(index,:) = gyro(index,:) - (C_Lframe2body*C_LN*(w_earth_value + w_transportation_value))';

    w_x = gyro(index,1);w_y = gyro(index,2);w_z = gyro(index,3);

    w_quat_matrix = [    0        -w_x     -w_y     -w_z;...
                        w_x        0        w_z     -w_y;...
                        w_y       -w_z      0        w_x;...
                        w_z        w_y     -w_x      0   ];

    attitude_quat_dot = 0.5*w_quat_matrix*quat_vector(index,:)';

    %--> Advance quaternion
    quat_vector(index+1,:) = quat_vector(index,:) + attitude_quat_dot'*sampling_period_sec;
    
    %--> Normalize the quaternion
    quat_vector(index+1,:) = quat_vector(index+1,:)./norm(quat_vector(index+1,:));
    
    [Euler_azimuth_value(index+1),Euler_pitch_value(index+1),Euler_roll_value(index+1)] = quat2angle(quat_vector(index+1,:),'ZYX');
    
    %--> Advance velocity
    v_N = [v_east_value(index); v_north_value(index); v_up_value(index)];
    v_N_dot_value = C_NL * C_body2Lframe * acc(index,:)' + g_N - cross((w_transportation_value + 2*w_earth_value),v_N);
    %v_N_dot_value = C_NL * C_body2Lframe * acc(index,:)' + g_N ;% Test the impact of neglecting the Coriolis effect 
    v_N_next = v_N + v_N_dot_value*sampling_period_sec;
    v_east_value(index+1) = v_N_next(1);
    v_north_value(index+1) = v_N_next(2); 
    v_up_value(index+1) = v_N_next(3);
    
    %--> Advance position
    position = [P_east(index); P_north(index); alt_value(index)];
    position_dot = v_N - cross(w_transportation_value,position);
    position_next = position + position_dot*sampling_period_sec;
    P_east(index+1) = position_next(1);
    P_north(index+1) = position_next(2);
    alt_value(index+1) = position_next(3);
    
    %--> Convert to the geodetic position
    [lat,lon,alt] = enu2geodetic(P_east(index+1),P_north(index+1),alt_value(index+1),lat0,lon0,alt0,wgs84Ellipsoid); 
    lon_value(index+1) = lon;
    lat_value(index+1) = lat;
end

NumRecords = num_of_samples;
time_vector_sec = ems_data.time(1:num_of_samples);

%--> Plot error values
v_east_ref_vector       = ems_data.vel_N(1:NumRecords,1)';
v_north_ref_vector      = ems_data.vel_N(1:NumRecords,2)';
v_up_ref_vector         = ems_data.vel_N(1:NumRecords,3)';
p_east_ref_vector       = ems_data.east(1:NumRecords);
p_north_ref_vector      = ems_data.north(1:NumRecords);
alt_ref_vector          = ems_data.h(1:NumRecords);
roll_ref_vector         = ems_data.roll(1:NumRecords);
pitch_ref_vector        = ems_data.pitch(1:NumRecords);
heading_ref_vector      = ems_data.heading(1:NumRecords);

ve_error = v_east_ref_vector'-v_east_value;
vn_error = v_north_ref_vector'-v_north_value;
vu_error = v_up_ref_vector'-v_up_value;
E_error = p_east_ref_vector-P_east;
N_error = p_north_ref_vector-P_north;
U_error = alt_ref_vector-alt_value;
roll_error = angdiff(roll_ref_vector,Euler_roll_value)*R2D;
pitch_error = angdiff(pitch_ref_vector,Euler_pitch_value)*R2D;
heading_error = angdiff(heading_ref_vector,Euler_azimuth_value)*R2D;

plot_processing_results
plot_position_curves
print_error_values
plot_error_curves