function [Alon, Blon, Clon, Dlon, XU0] = get_longitudinal_dynamics(tiltwing, XEQ, rho, g)
% *************************************************************************
% function [Alon, Blon, Clon, Dlon, XU0] = get_longitudinal_dynamics(tiltwing, XEQ, rho, g)
%
% This script takes input of an aero/propulsive model (aircraft) either strip theory or
% polynomial, particular trim condition states (XEQ), and some
% Earth constants and outputs a linearized state-space longitudinal model 
% (expressed in the body frame specified in Ref 1.)
% 
%  dx = A*x + B*u
%   y = C*x + D*u
%
%   x = [ u w th q ]'
%   u = [ oml omt omp dele ]'
%   y = [ u w th q Nz ]'

% Definitions:
%    u - velocity in the body x direction
%    w - velocity in the body z direction
%   th - pitch angle
%    q - pitch rate
%   Nz - Normal acceleration

%   oml - leading edge rotor speed
%   omt - trailing edge rotor speed
%   omp - pusher propeller speed
%   dele - elevator deflection 
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
%   Alon: linearized state-space longitudinal A matrix
%   Blon: linearized state-space longitudinal B matrix
%   Clon: linearized state-space longitudinal C matrix
%   Dlon: linearized state-space longitudinal D matrix
%   XU0:  trim vector (order required by GUAM/GVS)
% OTHER UTILIZED FUNCTIONS:
%   None.
% *************************************************************************

% VERSION HISTORY
% 4.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS
% 
% 7.20.2023, Michael J. Acheson (NASA LaRC D-316): Updated version to
% include documentation

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
tiltwing.aero(rho, vb0, om0, 1);

% Retrieve the relevant derivatives
idx_delf = 1;
idx_dela = 2;
idx_dele = 4;
idx_delr = 5;
idx_oml  = [ 7 11 15 19];
idx_omt   = [23 27 31 35];
idx_omp   = 39;

dFxdu = tiltwing.Fx_x(1);
dFzdu = tiltwing.Fz_x(1);
dMydu = tiltwing.My_x(1);

dFxdw = tiltwing.Fx_x(3);
dFzdw = tiltwing.Fz_x(3);
dMydw = tiltwing.My_x(3);

dFxdq = tiltwing.Fx_x(5);
dFzdq = tiltwing.Fz_x(5);
dMydq = tiltwing.My_x(5);

dFxdoml = sum(tiltwing.Fx_u(idx_oml));
dFzdoml = sum(tiltwing.Fz_u(idx_oml));
dMydoml = sum(tiltwing.My_u(idx_oml));

dFxdomt = sum(tiltwing.Fx_u(idx_omt));
dFzdomt = sum(tiltwing.Fz_u(idx_omt));
dMydomt = sum(tiltwing.My_u(idx_omt));

dFxdomp = tiltwing.Fx_u(idx_omp);
dFzdomp = tiltwing.Fz_u(idx_omp);
dMydomp = tiltwing.My_u(idx_omp);

dFxddele = tiltwing.Fx_u(idx_dele);
dFzddele = tiltwing.Fz_u(idx_dele);
dMyddele = tiltwing.My_u(idx_dele);

% Build the linear system for the longitudinal dynamics 
m  = tiltwing.mass;
Jy = tiltwing.I(2,2);
q  = om0(2);
th = alf0+gam0;
u  = vb0(1);
w  = vb0(3);

% location of accelerometer
x_a = 0.0; % at the center of mass

%         u             w             theta          q
%      ------------------------------------------------------------
Alon = [  1/m*dFxdu  -q+1/m*dFxdw  -g*cos(th)  -w+1/m*dFxdq    ; % | u
        q+1/m*dFzdu     1/m*dFzdw  -g*sin(th)   u+1/m*dFzdq    ; % | w
            0             0              0             1       ; % | th
         1/Jy*dMydu    1/Jy*dMydw        0          1/Jy*dMydq]; % | q

%         oml            omt           omp            dele 
%      ------------------------------------------------------------
Blon = [ 1/m*dFxdoml    1/m*dFxdomt   1/m*dFxdomp    1/m*dFxddele ; % | u 
         1/m*dFzdoml    1/m*dFzdomt   1/m*dFzdomp    1/m*dFzddele ; % | w 
            0             0              0             0       ; % | th
        1/Jy*dMydoml   1/Jy*dMydomt  1/Jy*dMydomp   1/Jy*dMyddele]; % | q 

%                u                       w                    theta        q
%      -------------------------------------------------------------------------------------------
Clon = [         1                       0                    0            0                   ; % | u 
                 0                       1                    0            0                   ; % | w 
                 0                       0                    1            0                   ; % | theta
                 0                       0                    0            1                   ; % | q
-(1/m*dFzdu-1/Jy*dMydu*x_a)/g  -(1/m*dFzdw-1/Jy*dMydw*x_a)/g sin(th)  -(1/m*dFzdq-1/Jy*dMydq*x_a)/g ; % | Nz 
           (1/m*dFxdu)/g            (1/m*dFxdw)/g           -cos(th)   (1/m*dFxdq)/g           ]; % | Nx 
            

%                 oml                              omt                               omp                                 dele
%      ------------------------------------------------------------------------------------------------------------------------------------
Dlon = [          0                               0                                0                                  0                ; % | u
                  0                               0                                0                                  0                ; % | w 
                  0                               0                                0                                  0                ; % | theta 
                  0                               0                                0                                  0                ; % | q 
-(1/m*dFzdoml-1/Jy*dMydoml*x_a)/g  -(1/m*dFzdomt-1/Jy*dMydomt*x_a)/g  -(1/m*dFzdomp-1/Jy*dMydomp*x_a)/g  -(1/m*dFzddele-1/Jy*dMyddele*x_a)/g ; % | Nz 
            (1/m*dFxdoml)/g                  (1/m*dFxdomt)/g                   (1/m*dFxdomp)/g                   (1/m*dFxddele)/g        ]; % | Nx

% SYS = ss(Alon, Blon, Clon, Dlon);
