function [Alat, Blat, Clat, Dlat, XU0] = get_lateral_dynamics(tiltwing, XEQ, rho, g)
% *************************************************************************
% function [Alat, Blat, Clat, Dlat, XU0] = get_lat_dynamics_heading(tiltwing, XEQ, rho, g)
%
% This script takes input of an aero/propulsive model (aircraft) either strip theory or
% polynomial, particular trim condition states (XEQ) and effectors (UEQ), 
% the number of aero surfaces (NS) and number of rotor/propeller surfaces (NP), and some
% Earth constants and outputs a linearized state-space lateral model 
% (expressed in the body frame specified in Ref 1.)
% 
%  dx = A*x + B*u
%   y = C*x + D*u
%
%   x = [ v p r phi ]'
%   u = [ dela delr]'
%   y = [ v p r phi ps rs ]'

% definitions:
%    v - velocity in the body y direction
%    p - roll rate
%    r - yaw rate
%  phi - roll angle
%  psi - yaw angle
%   ps - stability axis roll rate
%   rs - stability axis yaw rate 

%   dela - aileron deflection
%   delr - rudder deflection
%
% REFERENCES: 
% Ref 1 (unified controller): "Examination of Unified Control Approaches Incorporating
% Generalized Control Allocation", AIAA 2021-0999
%
% Ref 2 (strip theory aero/propulsive model): "A Strip Theory Approach to
% Dynamic Modeling of Tiltwing eVTOL Aircraft", AIAA 2021-1720
% 
% Ref 3 (polynomial aero/propulsive model): "Full-Envelope Aero-Propulsive Model
% Identification for Lift+Cruise Aircraft Using Computational Experiments",
% AIAA 2021-3170
%
% Written by Jacob Cook, NASA Langley Research Center.
% Current contact michael.j.acheson@nasa.gov, (757)-864-9457
% Dynamics Systems and Control Branch (DSCB D-316)
%
% INPUTS:
%   aircraft: Either matlab aircraft class object (strip theory), or
%       structure of L+C data (polynomial)
%   XEQ: Trim condition state vector, order:
%   rho: scalar value of Earth air density (slugs/ft^3)
%   grav: scalar value of Earth gravity acceleration (ft/sec^2)
% OUTPUTS:
%   Alat: linearized state-space lateral A matrix
%   Blat: linearized state-space lateral B matrix
%   Clat: linearized state-space lateral C matrix
%   Dlat: linearized state-space lateral D matrix
%   XU0:  trim vector (order required by GUAM/GVS)
% OTHER UTILIZED FUNCTIONS:
%   None.
% *************************************************************************

% VERSION HISTORY
% 4.15.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS

% Load in the equilibrium state
V0    = XEQ(1);
gam0  = XEQ(2);
R0    = XEQ(3);
alf0  = XEQ(4);
phi0  = XEQ(5);
th0   = gam0+alf0;
p0    = XEQ(6); 
q0    = XEQ(7); 
r0    = XEQ(8); 
omp0  = XEQ(9);
dele0 = XEQ(10);
dela0 = XEQ(11);
delr0 = XEQ(12);
oml0  = XEQ(13);
omt0  = XEQ(14);

% compute body velocities
vb0 = [V0*cos(alf0); 0; V0*sin(alf0)];
om0 =  [p0; q0; r0];



% body accelerations
ab0 = [ -g*sin(th0); g*cos(th0)*sin(phi0); g*cos(th0)*cos(phi0) ];

eta0 = [ phi0; th0; 0];

% not using flaps at the moment 
delf0 = 0;

XU0 = [vb0; om0; ab0; eta0; delf0; dela0; dele0; delr0; repmat(oml0,4,1); repmat(omt0,4,1); omp0];

% set the control inputs according to the desired trim conditions
tiltwing.om_p = zeros(1,9);
tiltwing.om_p(1:4) = oml0;
tiltwing.om_p(5:8) = omt0;
tiltwing.om_p(9)   = omp0;
tiltwing.del_e     = dele0;
tiltwing.del_a     = dela0;
tiltwing.del_r     = delr0;

% evaluate the aerodynamics of the aircraft and 
tiltwing.aero(rho, vb0, om0,1);

% Retrieve the relevant derivatives
idx_delf = 1;
idx_dela = 2;
idx_dele = 4;
idx_delr = 5;
idx_om_yaw  = [ 11 15 27 31 ];
idx_om_roll = [ 7 19 23 35 ];
idx_oml   = [ 7 11 15 19];
idx_omt   = [23 27 31 35];
idx_omp   = 39;

dFydv = tiltwing.Fy_x(2);
dMxdv = tiltwing.Mx_x(2);
dMzdv = tiltwing.Mz_x(2);

dFydp = tiltwing.Fy_x(4);
dMxdp = tiltwing.Mx_x(4);
dMzdp = tiltwing.Mz_x(4);

dFydr = tiltwing.Fy_x(6);
dMxdr = tiltwing.Mx_x(6);
dMzdr = tiltwing.Mz_x(6);

dFyddela = tiltwing.Fy_u(idx_dela);
dMxddela = tiltwing.Mx_u(idx_dela);
dMzddela = tiltwing.Mz_u(idx_dela);

dFyddelr = tiltwing.Fy_u(idx_delr);
dMxddelr = tiltwing.Mx_u(idx_delr);
dMzddelr = tiltwing.Mz_u(idx_delr);

dFydom_yaw  = tiltwing.Fy_u(idx_om_yaw)*[-1 1 1 -1]';
dMxdom_yaw  = tiltwing.Mx_u(idx_om_yaw)*[-1 1 1 -1]';
dMzdom_yaw  = tiltwing.Mz_u(idx_om_yaw)*[-1 1 1 -1]';

dFydom_roll = tiltwing.Fy_u(idx_om_roll)*[-1 1 -1 1]';
dMxdom_roll = tiltwing.Mx_u(idx_om_roll)*[-1 1 -1 1]';
dMzdom_roll = tiltwing.Mz_u(idx_om_roll)*[-1 1 -1 1]';

% Build the linear system for the longitudinal dynamics 
m   = tiltwing.mass;
Jx  = tiltwing.I(1,1);
Jz  = tiltwing.I(3,3);
Jxz = tiltwing.I(1,3);
p   = om0(1);
q   = om0(2);
r   = om0(3);

phi = eta0(1);
th  = eta0(2);
u   = vb0(1);
v   = vb0(2);
w   = vb0(3);

Jb = Jxz^2 - Jx*Jz;

%         v                           p                           r                          phi                
%      ------------------------------------------------------------------------------------------------------------
Alat  = [ 1/m*dFydv                   w+1/m*dFydp                -u+1/m*dFydr                g*cos(phi)*cos(th) ; % | v
          1/Jb*(-Jz*dMxdv+Jxz*dMzdv)  1/Jb*(-Jz*dMxdp+Jxz*dMzdp)  1/Jb*(-Jz*dMxdr+Jxz*dMzdr) 0                  ; % | p 
          1/Jb*(Jxz*dMxdv-Jx*dMzdv)   1/Jb*(Jxz*dMxdp-Jx*dMzdp)   1/Jb*(Jxz*dMxdr-Jx*dMzdr)  0                  ; % | r 
          0                           1                           cos(phi)*tan(th)          -r*sin(phi)*tan(th)]; % | phi  

%        om_roll                                 om_yaw                               dela                              delr
%      ---------------------------------------------------------------------------------------------------------------------------------------------
Blat = [ 1/m*dFydom_roll                         1/m*dFydom_yaw                       1/m*dFyddela                      1/m*dFyddelr                      ; % | v 
         1/Jb*(-Jz*dMxdom_roll+Jxz*dMzdom_roll)  1/Jb*(-Jz*dMxdom_yaw+Jxz*dMzdom_yaw) 1/Jb*(-Jz*dMxddela+Jxz*dMzddela)  1/Jb*(-Jz*dMxddelr+Jxz*dMzddelr)  ; % | p  
         1/Jb*(Jxz*dMxdom_roll-Jx*dMzdom_roll)   1/Jb*(Jxz*dMxdom_yaw-Jx*dMzdom_yaw)  1/Jb*(Jxz*dMxddela-Jx*dMzddela)   1/Jb*(Jxz*dMxddelr-Jx*dMzddelr)   ; % | r 
         0                                       0                                    0                                 0                                ]; % | phi  

%         v    p         r          phi 
%      --------------------------------------
Clat = [  1    0         0          0   ; % | v 
          0    1         0          0   ; % | p
          0    0         1          0   ; % | r  
          0    0         0          1   ; % | phi  
          0    cos(alf0) sin(alf0)  0   ; % | ps
          0   -sin(alf0) cos(alf0)  0  ]; % | rs
            
%         om_roll  om_yaw  dela  delr 
%      ----------------------------------
Dlat = [  0        0       0     0   ; % | v
          0        0       0     0   ; % | p
          0        0       0     0   ; % | r
          0        0       0     0   ; % | phi 
          0        0       0     0   ; % | ps 
          0        0       0     0  ]; % | rs 


% SYS = ss(Alat, Blat, Clat, Dlat);
