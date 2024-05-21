
ftlbs2watts = 1.355817948;

time = SimOut.Time.Data;

% Propeller properties
prop_torq = squeeze(SimOut.Vehicle.FM.Propulsion.Mprop_r.Data);
prop_thrust = squeeze(SimOut.Vehicle.FM.Propulsion.Fprop_r.Data);
prop_om = squeeze(SimOut.Vehicle.PropAct.EngSpeed.Data)';
prop_power_watts = (abs(prop_torq.*prop_om))*ftlbs2watts;


gamma = squeeze(SimOut.Vehicle.Sensor.gamma.Data);
psi   = squeeze(SimOut.Vehicle.Sensor.Euler.psi.Data);
theta   = squeeze(SimOut.Vehicle.Sensor.Euler.theta.Data);
phi   = squeeze(SimOut.Vehicle.Sensor.Euler.phi.Data);


% Actuators
dele = squeeze(SimOut.Vehicle.SurfAct.Position.Data(3,1,:));
dela = squeeze(SimOut.Vehicle.SurfAct.Position.Data(1,1,:));
delr = squeeze(SimOut.Vehicle.SurfAct.Position.Data(5,1,:));

d_dele = squeeze(SimOut.Vehicle.SurfAct.Rate.Data(3,1,:));
d_dela = squeeze(SimOut.Vehicle.SurfAct.Rate.Data(1,1,:));
d_delr = squeeze(SimOut.Vehicle.SurfAct.Rate.Data(5,1,:));

% Reference Commands
pos_des  = squeeze(SimOut.RefInputs.pos_des.Data)';
vel_des  = squeeze(SimOut.RefInputs.Vel_bIc_des.Data)';
chi_des  = squeeze(SimOut.RefInputs.chi_des.Data);
dchi_des = squeeze(SimOut.RefInputs.chi_dot_des.Data);

% vehicle position in inertial space
pos = squeeze(SimOut.Vehicle.EOM.InertialData.Pos_bii.Data)';
vel_bIi = squeeze(SimOut.Vehicle.EOM.InertialData.Vel_bIi.Data)'; 
for ii = 1:length(time)
  qChi = Euler2quat(0,0,chi_des(ii)*180/pi,'123');
  vel_bIc(ii,:) = quatrotate(qChi', vel_bIi(ii,:));
  chi(ii) = atan2(vel_bIi(ii,2), vel_bIi(ii,1));
end

% Winds
%winds = squeeze(SimOut.Env.Wind.Vel_wHh.Data);


% Control Surfaces
figure
subplot(3,1,1)
plot(time, [dele d_dele]*180/pi,'linewidth',1.25)
title('{\bf Control Surfaces}','fontsize',15)
ylabel('elevator','fontsize',15)
legend({'$\delta$ [deg]','$\dot{\delta}$ [deg/sec]'}, 'fontsize', 12)
grid on;
zoom on;

subplot(3,1,2)
plot(time, [dela d_dela]*180/pi,'linewidth',1.25)
ylabel('aileron','fontsize',15)
grid on;
zoom on;

subplot(3,1,3)
plot(time, [delr d_delr]*180/pi,'linewidth',1.25)
ylabel('rudder','fontsize',15)
xlabel('time [sec]','fontsize',15)
grid on;
zoom on;

figure
subplot(3,1,1)
plot(time, [dele]*180/pi,'linewidth',1.25)
title('{\bf Control Surfaces}','fontsize',15)
ylabel('elevator','fontsize',15)
grid on;
zoom on;

subplot(3,1,2)
plot(time, [dela]*180/pi,'linewidth',1.25)
ylabel('aileron','fontsize',15)
grid on;
zoom on;

subplot(3,1,3)
plot(time, [delr]*180/pi,'linewidth',1.25)
ylabel('rudder','fontsize',15)
xlabel('time [sec]','fontsize',15)
grid on;
zoom on;

% rotor/prop plot 3X1
figure
subplot(2,1,1)
plot(time, prop_om(:,1:8)*60/2/pi,'linewidth',1.25)
title('{\bf Lifting Rotors and Pusher Prop}','fontsize',15)
ylabel('rotors [rpm]','fontsize',15)
grid on
zoom on

subplot(2,1,2)
plot(time, prop_om(:,9)*60/2/pi,'linewidth',1.25)
ylabel('pusher [rpm]','fontsize',15)
xlabel('time [sec]','fontsize',15)
grid on
zoom on


% Combined plot
figure
plot(time, [pos],'linewidth',1.25)
title('{\bf Position in Inertial Frame}','fontsize',15);
ylabel('feet','fontsize',15)
xlabel('time [sec]','fontsize',15)
legend({'x','y','z'},'fontsize',12)
axis([0 60 -35 5]);
grid on
zoom on

figure
subplot(3,1,1)
plot(time, [pos(:,1) pos_des(:,1)],'linewidth',1.25)
title('{\bf Position in Inertial Frame}','fontsize',15);
ylabel('$x$ [ft]','fontsize',15)
legend({'flown','desired'},'fontsize',12)
grid on
zoom on
subplot(3,1,2)
plot(time, [pos(:,2) pos_des(:,2)],'linewidth',1.25)
ylabel('$y$ [ft]','fontsize',15)
grid on
zoom on
subplot(3,1,3)
plot(time, [pos(:,3) pos_des(:,3)],'linewidth',1.25)
ylabel('$z$ [ft]','fontsize',15)
xlabel('time [sec]','fontsize',15)
grid on
zoom on

figure
subplot(3,1,1)
plot(time, [vel_bIc(:,1) vel_des(:,1)],'linewidth',1.25)
title('{\bf Heading Frame Velocity}','fontsize',15);
ylabel('$\bar{u}$ [ft/s]','fontsize',15)
legend({'flown','desired'},'fontsize',12)
grid on
zoom on
subplot(3,1,2)
plot(time, [vel_bIc(:,2) vel_des(:,2)],'linewidth',1.25)
ylabel('$\bar{v}$ [ft/s]','fontsize',15)
grid on
zoom on
subplot(3,1,3)
plot(time, [vel_bIc(:,3) vel_des(:,3)],'linewidth',1.25)
ylabel('$\bar{w}$ [ft/s]','fontsize',15)
xlabel('time [sec]','fontsize',15)
grid on
zoom on

% figure
% plot(time, winds,'linewidth',1.25)
% ylabel('{\bf Winds [ft/s]}','fontsize',15)
% xlabel('time [sec]','fontsize',15)
% legend('$u_w$','$v_w$','$w_w$');
% grid on
% zoom on

figure
plot(time, [theta phi]*180/pi,'linewidth',1.25)
title('{\bf Pitch and Roll Angles}','fontsize',15);
ylabel('{[deg]}','fontsize',15)
xlabel('time [sec]','fontsize',15)
legend('$\theta$','$\phi$');
grid on
zoom on



