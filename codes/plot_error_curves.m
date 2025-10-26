%--> Plot error curves
figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),E_error(1:NumRecords),'b','LineWidth',2);grid on;
title('east position error (m)');xlabel('time(s)');ylabel('east position error (m)');
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),N_error(1:NumRecords),'b','LineWidth',2);grid on;
title('north position error (m)');xlabel('time(s)');ylabel('north position error (m)');
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),U_error(1:NumRecords),'b','LineWidth',2);grid on;
title('altitude position error (m)');xlabel('time(s)');ylabel('altitude position error (m)');

figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),ve_error(1:NumRecords),'b','LineWidth',2);grid on;
title('east velocity error (m/s)');xlabel('time(s)');ylabel('east velocity error (m/s)');
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),vn_error(1:NumRecords),'b','LineWidth',2);grid on;
title('north velocity error (m/s)');xlabel('time(s)');ylabel('north velocity error (m/s)');
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),vu_error(1:NumRecords),'b','LineWidth',2);grid on;
title('up velocity error (m/s)');xlabel('time(s)');ylabel('up velocity error (m/s)');

figure;
subplot(3,1,1);
plot(time_vector_sec(1:NumRecords),roll_error(1:NumRecords),'b','LineWidth',2);grid on;
title('roll error (deg)');xlabel('time(s)');ylabel('roll error (deg)');
subplot(3,1,2);
plot(time_vector_sec(1:NumRecords),pitch_error(1:NumRecords),'b','LineWidth',2);grid on;
title('pitch error (deg)');xlabel('time(s)');ylabel('pitch error (deg)');
subplot(3,1,3);
plot(time_vector_sec(1:NumRecords),heading_error(1:NumRecords),'b','LineWidth',2);grid on;
title('azimuth error (deg)');xlabel('time(s)');ylabel('azimuth error (deg)');
