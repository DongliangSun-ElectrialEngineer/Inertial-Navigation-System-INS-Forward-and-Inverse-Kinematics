
figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),ems_data.east(1:NumRecords),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),P_east(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('east position (m)');xlabel('time(s)');ylabel('east position (m)')
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),ems_data.north(1:NumRecords),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),P_north(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('north position (m)');xlabel('time(s)');ylabel('north position (m)')
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),ems_data.h(1:NumRecords),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),alt_value(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('up position (m)');xlabel('time(s)');ylabel('up position (m)')

figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,1),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),v_east_value(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('east velocity (m/s)');xlabel('time(s)');ylabel('east velocity (m/s)')
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,2),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),v_north_value(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('north velocity (m/s)');xlabel('time(s)');ylabel('north velocity (m/s)')
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,3),'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),v_up_value(1:NumRecords),'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('up velocity (m/s)');xlabel('time(s)');ylabel('up velocity (m/s)')

figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),ems_data.roll(1:NumRecords)*R2D,'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_roll_value(1:NumRecords)*R2D,'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('roll (deg)');xlabel('time(s)');ylabel('roll (deg)')
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),ems_data.pitch(1:NumRecords)*R2D,'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_pitch_value(1:NumRecords)*R2D,'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('picth (deg)');xlabel('time(s)');ylabel('picth (deg)')
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),ems_data.heading(1:NumRecords)*R2D,'g','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_azimuth_value(1:NumRecords)*R2D,'r','LineWidth',1);grid on;
legend('ground-truth','processed');
title('heading (deg)');xlabel('time(s)');ylabel('heading (deg)')