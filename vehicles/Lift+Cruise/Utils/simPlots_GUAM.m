SimOut = logsout{1}.Values;
ftlbs2watts = 1.355817948;

time = SimOut.Time.Data;

% Propeller properties
prop_torq = SimOut.Vehicle.FM.Propulsion.Mprop_r.Data;
prop_thrust = SimOut.Vehicle.FM.Propulsion.Fprop_r.Data;
prop_om = SimOut.Vehicle.PropAct.EngSpeed.Data;
prop_power_watts = (abs(prop_torq.*prop_om))*ftlbs2watts;

% reference tracking
if SimIn.refInputType == RefInputEnum.TIMESERIES
    V_cmd     = SimIn.RefInputs.trajectory.Vel_bIc_des.Data(:,1);
%     gamma_cmd = SimIn.RefInputs.trajectory.gamma;
%     chi_cmd   = SimIn.RefInputs.trajectory.chi;
else 
    V_cmd     = SimOut.RefInputs.Vel_bIc_des.Data(:,1);
end

V     = SimOut.Vehicle.Sensor.Vtot.Data;
gamma = SimOut.Vehicle.Sensor.gamma.Data;
psi   = SimOut.Vehicle.Sensor.Euler.psi.Data;
theta = SimOut.Vehicle.Sensor.Euler.theta.Data;
phi   = SimOut.Vehicle.Sensor.Euler.phi.Data;


% Actuators
dele = SimOut.Vehicle.SurfAct.Position.Data(:,3);
dela = SimOut.Vehicle.SurfAct.Position.Data(:,1);
delr = SimOut.Vehicle.SurfAct.Position.Data(:,5);

d_dele = SimOut.Vehicle.SurfAct.Rate.Data(:,3);
d_dela = SimOut.Vehicle.SurfAct.Rate.Data(:,1);
d_delr = SimOut.Vehicle.SurfAct.Rate.Data(:,5);

if SimIn.refInputType == RefInputEnum.TIMESERIES
    pos_des     = SimOut.RefInputs.pos_des.Data;
    vel_des     = SimIn.RefInputs.trajectory.vel_des; % NED frame?
    t_des       = SimIn.RefInputs.trajectory.refTime;
else
    pos_des     = SimOut.RefInputs.pos_des.Data;
    % Compute inertial velocity from heading frame velocity and desired heading
    q           = QrotZ(-SimOut.RefInputs.chi_des.Data);
    vel_des     = Qtrans(q,SimOut.RefInputs.Vel_bIc_des.Data);
    t_des       = time;
end

pos = squeeze(SimOut.Vehicle.EOM.InertialData.Pos_bii.Data);

% Winds
winds = squeeze(SimOut.Env.Wind.Vel_wHh.Data);

figure
plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3),'-','linewidth',1.5)
hold on
plot3(pos_des(:,1), pos_des(:,2), 0*pos_des(:,3),'--','linewidth',1.5)
plot3(pos(:,1), pos(:,2), pos(:,3),'-','linewidth',1.5)
plot3(pos(:,1), pos(:,2), 0*pos(:,3),'--','linewidth',1.5)
legend({'path','ground track','flown path','flown ground track'},'fontsize',12)
set(gca,'Ydir','reverse')
set(gca,'Zdir','reverse')
xlabel('{\bf North [ft]}','fontsize',15)
ylabel('{\bf East [ft]}','fontsize',15)
zlabel('{\bf height [ft]}','fontsize',15)
spc     = 100;
if max(pos(:,3)) >= 0
    spc_z   = (max(pos(:,3))-min(pos(:,3)))/10;
else
    spc_z   = -(max(pos(:,3))-min(pos(:,3)));
end
mx1     = max([max(pos(:,1)), max(pos_des(:,1)), 0]);
mn1     = min([min(pos(:,1)), min(pos_des(:,1)), 0]);
spc1    = (mx1-mn1)/10;
mx2     = max([max(pos(:,2)), max(pos_des(:,2)), 0]);
mn2     = min([min(pos(:,2)), min(pos_des(:,2)), 0]);
spc2    = (mx2-mn2)/10;
mx3     = max([max(pos(:,3)), max(pos_des(:,3)), 0]);
mn3     = min([min(pos(:,3)), min(pos_des(:,3)), 0]);
spc3    = (mx3-mn3)/10;

axis([mn1-spc1 mx1+spc1 mn2-spc2 mx2+spc2 mn3-spc3 mx3+spc3])
grid on
view(6,33)

figure
plot(pos_des(:,2), pos_des(:,1),'-','linewidth',1.5)
hold on
plot(pos(:,2), pos(:,1),'-','linewidth',1.5)
xlabel('{\bf East}','Interpreter','latex')
ylabel('{\bf North}','Interpreter','latex')
legend({'ground track','flown ground track'},'fontsize',12)
grid on
zoom on


% Control Surfaces
figure
subplot(3,1,1)
plot(time, [dele d_dele]*180/pi)
ylabel('elevator','fontsize',15)
legend({'$\delta$ [deg]','$\dot{\delta}$ [deg/sec]'}, 'fontsize', 12,'Interpreter','latex')
axis([min(time) max(time) -35 35])
grid on;
zoom on;

subplot(3,1,2)
plot(time, [dela d_dela]*180/pi)
ylabel('aileron','fontsize',15)
axis( [min(time) max(time) -35 35])
grid on;
zoom on;

subplot(3,1,3)
plot(time, [delr d_delr]*180/pi)
ylabel('rudder','fontsize',15)
xlabel('time [sec]','fontsize',15)
axis( [min(time) max(time) -35 35])
grid on;
zoom on;

figure
plot(time, [phi theta]*180/pi)
title('Pitch and Roll Angles')
ylabel('[deg]','fontsize',15)
legend({'$\phi$','$\theta$'},'fontsize',12,'Interpreter','latex')
axis([min(time) max(time) -40 40])
grid on;
zoom on;

% Control Surfaces
figure
subplot(2,1,1)
plot(time, [dele]*180/pi)
ylabel('elevator [deg]','fontsize',15)
axis([min(time) max(time) -40 40])
grid on;
zoom on;

subplot(2,1,2)
plot(time, prop_om(:,1:8)*60/2/pi)
ylabel('rotors [rpm]','fontsize',15)
axis([0 max(time) 0 1500])
grid on
zoom on

% performance plots
figure
subplot(4,1,1)
plot(time, -pos(:,3),time, -pos_des(:,3))
ylabel('height [ft]','fontsize',15)
legend({'flown','desired'},'fontsize',12)
%axis([min(t_des) max(time) min(-pos_des(:,3)) max(-pos_des(:,3))+spc])
grid on
zoom on

subplot(4,1,2)
if SimIn.refInputType == RefInputEnum.TIMESERIES
    plot(time, V,t_des, V_cmd)
else
    plot(time, V)
end
ylabel('vel [ft/s]','fontsize',15)
%axis([min(t_des) max(time) min(V) max(V)+10])
grid on;
zoom on;

subplot(4,1,3)
plot(time, gamma*180/pi) %, t_des, gamma_cmd*180/pi)
ylabel('$\gamma$ [deg]','fontsize',15,'Interpreter','latex')
%axis([min(t_des) max(time) min(gamma)*180/pi-10 max(gamma)*180/pi+10])
grid on;
zoom on;

subplot(4,1,4)
plot(time, psi*180/pi) %, t_des, chi_cmd*180/pi)
ylabel('$\psi$ [deg]','fontsize',15,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
%axis([min(t_des) max(time) -25 25])
grid on;
zoom on;

total_power = sum(prop_power_watts,2);

% rotor/prop plot 4X1
figure
subplot(4,1,1)
plot(time, prop_om(:,1:4)*60/2/pi)
ylabel('leading [rpm]','fontsize',15)
legend({'rotor 1','rotor 2','rotor 3','rotor 4'},'fontsize',12)
%axis([0 max(time) 0 1250])
grid on
zoom on

subplot(4,1,2)
plot(time, prop_om(:,5:8)*60/2/pi)
ylabel('trailing [rpm]','fontsize',15)
legend({'rotor 5','rotor 6','rotor 7','rotor 8'},'fontsize',12)
%axis([0 max(time) 0 1250])
grid on
zoom on

subplot(4,1,3)
plot(time, prop_om(:,9)*60/2/pi)
ylabel('pusher [rpm]','fontsize',15)
%axis([0 max(time) 0 2500])
grid on
zoom on

subplot(4,1,4)
plot(time, total_power/1000,'linewidth',1.5)
ylabel('{\bf Power [kW]}','fontsize',15,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
%axis([-1 max(time) 0 1000])
grid on
zoom on

% rotor/prop plot 3X1
figure
subplot(3,1,1)
plot(time, prop_om(:,1:8)*60/2/pi)
ylabel('rotors [rpm]','fontsize',10)
%axis([0 max(time) 0 1250])
grid on
zoom on

subplot(3,1,2)
plot(time, prop_om(:,9)*60/2/pi)
ylabel('pusher [rpm]','fontsize',10)
%axis([0 max(time) 0 3000])
grid on
zoom on

subplot(3,1,3)
plot(time, total_power/1000,'linewidth',1.5)
ylabel('{\bf Power [kW]}','fontsize',10,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
%axis([-1 max(time) 0 1000])
grid on
zoom on

% Combined plot
figure
subplot(6,1,1)
plot(time, -pos(:,3),time, -pos_des(:,3))
ylabel('height [ft]','fontsize',15)
legend({'flown','desired'},'fontsize',12)
%axis([min(t_des) max(time) min(-pos_des(:,3)) max(-pos_des(:,3))+spc])
grid on
zoom on

subplot(6,1,2)
plot(time, V/SimIn.Units.knot,t_des, V_cmd/SimIn.Units.knot)
ylabel('vel [kts]','fontsize',15)
%axis([min(t_des) max(time) min(V)/SimIn.Units.knot max(V_cmd)/SimIn.Units.knot+25])
grid on;
zoom on;

% subplot(6,1,3)
plot(time, psi*180/pi) %, t_des, chi_cmd*180/pi)
ylabel('$\chi$ [deg]','fontsize',15,'Interpreter','latex')
%axis([min(t_des) max(time) -180 0])
grid on;
zoom on;

subplot(6,1,4)
plot(time, prop_om(:,9)*60/2/pi)
ylabel('pusher [rpm]','fontsize',15)
%axis([0 max(time) 0 3000])
grid on
zoom on

subplot(6,1,5)
plot(time, prop_om(:,1:8)*60/2/pi)
ylabel('rotors [rpm]','fontsize',15)
%axis([0 max(time) 0 1200])
grid on
zoom on

subplot(6,1,6)
plot(time, total_power/1000,'linewidth',1.5)
ylabel('{\bf Power [kW]}','fontsize',15,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
%axis([-1 max(time) 0 1000])
grid on
zoom on

% Combined plot
figure
subplot(6,1,1)
plot(time, -pos(:,3),time, -pos_des(:,3))
ylabel('height [ft]','fontsize',15)
legend({'flown','desired'},'fontsize',12)
axis([min(t_des) max(time) min(-pos_des(:,3)) max(-pos_des(:,3))+spc])
grid on
zoom on

subplot(6,1,2)
plot(time, V/SimIn.Units.knot,t_des, V_cmd/SimIn.Units.knot)
ylabel('vel [kts]','fontsize',15)
axis([min(t_des) max(time) min(V)/SimIn.Units.knot max(V_cmd)/SimIn.Units.knot+10])
grid on;
zoom on;

subplot(6,1,3)
plot(time, [dele]*180/pi)
ylabel('elevator [deg]','fontsize',15)
%axis([min(time) max(time) -40 40])
grid on;
zoom on;

subplot(6,1,4)
plot(time, prop_om(:,9)*60/2/pi)
ylabel('pusher [rpm]','fontsize',15)
%axis([0 max(time) 0 3000])
grid on
zoom on

subplot(6,1,5)
plot(time, prop_om(:,1:8)*60/2/pi)
ylabel('rotors [rpm]','fontsize',15)
%axis([0 max(time) 0 1200])
grid on
zoom on

subplot(6,1,6)
plot(time, total_power/1000,'linewidth',1.5)
ylabel('{\bf Power [kW]}','fontsize',15,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
%axis([-1 max(time) 0 1000])
grid on
zoom on

figure
plot(time, winds,'linewidth',1.25)
ylabel('{\bf Winds [ft/s]}','fontsize',15,'Interpreter','latex')
xlabel('time [sec]','fontsize',15)
legend('$u_w$','$v_w$','$w_w$','Interpreter','latex');
%axis([0 max(time) -25 25])
grid on
zoom on

figure
plot(time, (pos-pos_des),'linewidth',1.25)
ylabel('Position Error [ft]','fontsize',15)
xlabel('time [sec]','fontsize',15)
legend('x','y','z');
grid on
zoom on


