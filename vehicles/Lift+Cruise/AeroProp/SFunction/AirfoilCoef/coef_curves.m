

load('NACA_0015_ext.mat')


alf = NACA0015_ext(:,1);
cl = NACA0015_ext(:,2);
cd = NACA0015_ext(:,3);
cm = NACA0015_ext(:,4);

figure
plot(alf,[cl cd ])
xlabel('angle of attack [deg]')
ylabel('lift and drag coefficients')
grid on
zoom on

% figure
% plot(alf,cd)
% xlabel('angle of attack [deg]')
% ylabel('drag coefficient')
% grid on
% zoom on

cl_pp = spline(alf*pi/180,cl);
cd_pp = spline(alf*pi/180,cd);
cm_pp = spline(alf*pi/180,cm);

NACA_0015_pp.alf = alf*pi/180;
NACA_0015_pp.cl = cl_pp;
NACA_0015_pp.cd = cd_pp;
NACA_0015_pp.cm = cm_pp;

save('NACA_0015_pp.mat', 'NACA_0015_pp');
