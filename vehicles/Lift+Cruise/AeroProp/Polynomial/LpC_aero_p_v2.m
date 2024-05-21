function [X,Y,Z,L,M,N,Validity] = LpC_aero_p_v2(x,n,d,rho,a,blending_method,Units,Model)
% LpC_aero - Implements the Lift+Cruise aero model obtained from DOE tests
%
% DESCRIPTION: 
%   This script runs the aerodynamic model obtained from CFD results for
%   the Lift+Cruise vehicle.
%
% INPUTS: 
%   x - state vector containing body-axis velocity and body-axis angular 
%       rates, x=[u,v,w,p,q,r] in ft/s and rad/s
%   n - vector containing propeller speed values, n=[n1,n2...] in rad/s
%   d - vector control surface positions, n=[d1,d2...] in rad
%   rho - atmospheric density in slug/ft^3
%   a - speed of sound [ft/s]
%   blending_method - method of combining overlapping models
%         1=linear
%         2=smooth "sigmoid-like" blending (continuous and differentiable)
%   Units - SimIn.Units
%   Model - SimIn.Model
%
% OUTPUTS:
%   X - x body axis aerodynamic force, lbf
%   Y - y body axis aerodynamic force, lbf
%   Z - z body axis aerodynamic force, lbf
%   L - aerodynamic rolling moment, ft-lbf
%   M - aerodynamic pitching moment, ft-lbf
%   N - aerodynamic yawing moment, ft-lbf
%   Validity - 
%     invalid_speed - flag indicating invalid speed
%     invalid_prop_speed - flag indicating invalid prop speeds
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   Flight Dynamics Branch (D317)
%   NASA Langley Research Center
%
% REFERENCE:
%   Simmons, B. M., Buning, P. G., and Murphy, P. C., "Full-Envelope
%   Aero-Propulsive Model Identification for Lift+Cruise Aircraft Using
%   Computational Experiments," AIAA AVIATION Forum, Aug. 2021.
%   https://doi.org/10.2514/6.2021-3170
%
% HISTORY:
%   04 MAR 2021 - created and debugged, BMS
%

% extract states
u_fps=x(1);
v_fps=x(2);
w_fps=x(3);

p_rps=x(4);
q_rps=x(5);
r_rps=x(6);

V_fps=sqrt(u_fps^2+v_fps^2+w_fps^2);% freestream velocity [ft/s]
if u_fps>0.01
    Alpha=atand(w_fps/u_fps);% angle of attack [deg]
    Beta=asind(v_fps/V_fps);% angle of sideslip [deg]
else
    % set alpha and beta to zero at low airspeed
    Alpha=0;
    Beta=0;
end

% conversion constants
r2d=1/Units.deg;
fps2kts=1/Units.knot;
radps2rpm=60/(2*pi);

u=u_fps*fps2kts; % x body-axis velocity [kts]
v=v_fps*fps2kts; % y body-axis velocity [kts]
w=w_fps*fps2kts; % z body-axis velocity [kts]

LA=d(1)*r2d; % Left Aileron [deg]
RA=d(2)*r2d; % Right Aileron [deg]
LE=d(3)*r2d; % Left Elevator [deg]
RE=d(4)*r2d; % Right Elevator [deg]
RUD=d(5)*r2d; % Rudder [deg]

N1=n(1)*radps2rpm; % Engine 1 Speed [RPM]
N2=n(5)*radps2rpm; % Engine 2 Speed [RPM]
N3=n(2)*radps2rpm; % Engine 3 Speed [RPM]
N4=n(6)*radps2rpm; % Engine 4 Speed [RPM]
N5=n(3)*radps2rpm; % Engine 5 Speed [RPM]
N6=n(7)*radps2rpm; % Engine 6 Speed [RPM]
N7=n(4)*radps2rpm; % Engine 7 Speed [RPM]
N8=n(8)*radps2rpm; % Engine 8 Speed [RPM]
N9=n(9)*radps2rpm; % Engine 9 Speed [RPM]

% -------------------------------------------------------------------------
%  Lift+Cruise static aerodynamics
% -------------------------------------------------------------------------
[AeroModel, Validity]=LpC_interp_p_v2(V_fps,rho,a,u,v,w,Alpha,Beta,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9,blending_method,Model);
% Output: AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

% body-axis aerodynamic forces [lbf]
Xs=-1*AeroModel(1);
Ys=   AeroModel(2);
Zs=-1*AeroModel(3);

% body-axis aerodynamic moments [ft-lbf]
Ls=AeroModel(4);
Ms=AeroModel(5);
Ns=AeroModel(6);

% -------------------------------------------------------------------------
% Lift+Cruise aerodynamic damping (airframe only)
% -------------------------------------------------------------------------

% bare-airframe damping for the Lift+Cruise
[Xd,Yd,Zd,Ld,Md,Nd] = LpC_glider_damping(V_fps,rho,p_rps,q_rps,r_rps,Model);
% note that propulsion-damping and higher fidelity damping estimates
% including interaction effects will be added at a later date

% -------------------------------------------------------------------------
% Lift+Cruise isolated rotor/propeller calculations
% -------------------------------------------------------------------------

% propeller speed in rev/s
n_revps=[N1;N2;N3;N4;N5;N6;N7;N8;N9]/60;

% calculate the total forces and moments due to the rotors/propellers
[Xp_tot,Yp_tot,Zp_tot,Lp_tot,Mp_tot,Np_tot]=LpC_Total_PropFM(u_fps,v_fps,w_fps,p_rps,q_rps,r_rps,n_revps,rho,a,Model);

% calculate the total forces and moments due to the rotors/propellers
% assuming the vehicle has no angular velocity
[Xp_st,Yp_st,Zp_st,Lp_st,Mp_st,Np_st]=LpC_Total_PropFM(u_fps,v_fps,w_fps,0,0,0,n_revps,rho,a,Model);

% calculate the estimated effect of isolated rotor damping by subtracting
% the forces and moments calculated with and without body angular velocity
Xp_dyn=Xp_tot-Xp_st;
Yp_dyn=Yp_tot-Yp_st;
Zp_dyn=Zp_tot-Zp_st;
Lp_dyn=Lp_tot-Lp_st;
Mp_dyn=Mp_tot-Mp_st;
Np_dyn=Np_tot-Np_st;


% add the static and dynamic forces to calculate the total forces and
% moments in the body frame
X=Xs+Xd+Xp_dyn;
Y=Ys+Yd+Yp_dyn;
Z=Zs+Zd+Zp_dyn;
L=Ls+Ld+Lp_dyn;
M=Ms+Md+Mp_dyn;
N=Ns+Nd+Np_dyn;

end
