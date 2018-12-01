close all;

jointNum=1;

figure(1);
plot(d_Time,rad2deg(d_ActJPos(:,jointNum)),'r'); hold on; grid on;
plot(d_Time,rad2deg(d_DesJPos(:,jointNum)),'b');

figure(2)
plot(d_Time,rad2deg(d_ActJVel(:,jointNum)),'r'); hold on; grid on;
plot(d_Time,rad2deg(d_DesJVel(:,jointNum)),'b');

figure(3)
plot(d_Time,rad2deg(d_ActJAcc(:,jointNum)),'r'); hold on; grid on;
plot(d_Time,rad2deg(d_DesJAcc(:,jointNum)),'b');

figure(4)
plot(d_Time,d_DesTrq(:,jointNum),'b'); hold on; grid on;

figure(5)
plot(d_Time,rad2deg(d_ErrorJPos(:,jointNum)),'b'); hold on; grid on;

figure(6)
plot(d_Time,rad2deg(d_ErrorJVel(:,jointNum)),'b'); hold on; grid on;