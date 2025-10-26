figure;
plot(ems_data.east(1:NumRecords),ems_data.north(1:NumRecords),'LineWidth',2);
hold on;grid on;title('2-D Trajectory Plot');
plot(P_east(1:NumRecords),P_north(1:NumRecords),'LineWidth',2);
xlabel('East(m)');ylabel('North(m)');
legend('ground-truth','processed');

figure;
plot3(ems_data.east(1:NumRecords),ems_data.north(1:NumRecords),ems_data.h(1:NumRecords),'LineWidth',2);
hold on;grid on;title('3-D Trajectory Plot');
plot3(P_east(1:NumRecords),P_north(1:NumRecords),alt_value(1:NumRecords),'LineWidth',2);
legend('ground-truth','processed');
xlabel('East(m)');ylabel('North(m)');zlabel('Up(m)');
view(-12,10);