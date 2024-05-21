% Plot_PW_Bezier.m plots the ownship top level trajectory information.
% This includes 3D position trajectory curve and associated PW Bernstein
% polynomial velocity and acceleration profiles.  This file is used to
% visual a given PW Bernstein polynomial trajectory.

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 9.29.2023, MJA: Initial version of script.  Creates and plots ownship 
% PW Bernstein polynomial trajectory information 

% *************************************************************************
% Plots current trajectory
close all
p_int = 0.01;
waypoints = {wptsX, wptsY, wptsZ};
time_wpts = {time_wptsX, time_wptsY, time_wptsZ};
time = linspace(0,time_wptsX(end), time_wptsX(end)/p_int);
pwcurve_plot = genPWCurve(waypoints,time_wpts);

% Plot positions with time
values = evalPWCurve(pwcurve_plot,time,0);
%figure
figure(1);
plot3(values(:,1), values(:,2), values(:,3))
title('3D Desired trajectory');
hold all
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
values2 = evalPWCurve(pwcurve_plot,time_wptsX ,0);
plot3(values2(:,1), values2(:,2), values2(:,3),'rx');

% Add trajectory time at beginning of each PW segment position
for loop = 1:length(time_wptsX)
    text(values2(loop,1), values2(loop,2), values2(loop,3),sprintf('%0.1f',round(time_wptsX(loop))));
end
hold all
% Add segment number at beginning of each PW segment position
% for loop = 0:length(time_wptsX)-1
%     text(values2(loop+1,1), values2(loop+1,2), -values2(loop+1,3),sprintf('%0.1f',loop));
% end

%figure;
figure(2);
plot(time, values);
title('Positions');
xlabel('Time (sec)');
ylabel('Postion (ft)');
grid on;
legend('X', 'Y', 'Z');
hold all

%Plot velocities with time
values = evalPWCurve(pwcurve_plot,time,1);
%figure;
figure(3);
plot(time, values)
title('Velocities')
xlabel('Time (sec)');
ylabel('Velocity (ft/sec)');
grid on;
legend('X','Y','Z')
disp('')
hold all;

%Plot accelerations with time
values = evalPWCurve(pwcurve_plot,time,2);
%figure;
figure(4);
plot(time, values)
title('Accelerations')
xlabel('Time (sec)');
ylabel('Acc (ft/sec^2)');
grid on;
legend('X','Y','Z')
hold all