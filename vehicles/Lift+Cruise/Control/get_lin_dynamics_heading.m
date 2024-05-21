function [A, B, C, D, XU0] = get_lin_dynamics_heading(tiltwing, XEQ, UEQ, NS, NP, rho, g)
% *************************************************************************
% function [A, B, C, D, XU0] = get_lin_dynamics_heading(tiltwing, XEQ, UEQ, NS, NP, rho, g)
%
% This script takes input of an aero/propulsive model (aircraft) either strip theory or
% polynomial, particular trim condition states (XEQ) and effectors (UEQ), 
% the number of aero surfaces (NS) and number of rotor/propeller surfaces (NP), and some
% Earth constants and outputs a linearized state-space lateral model 
% (expressed in the control frame specified in Ref 1.)
% Outputs the system matrix for the linear model
% 
%  dx = A*x + B*u
%   y = C*x + D*u
%
%   x = [ px py pz phi th psi ubar vbar wbar p q r ]'
%   u = [ delf dela dele delr omp1-9]'
%   y = x 
% 
%  dx = A*x + B*u
%   y = C*x + D*u
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
%   UEQ: Trim condition effector vector, order: 
%   NS: scalar number of aerodynamic surfaces
%   NP: scalar number of rotor/prop effectors 
%   rho: scalar value of Earth air density (slugs/ft^3)
%   grav: scalar value of Earth gravity acceleration (ft/sec^2)
% OUTPUTS:
%   Alat: linearized state-space lateral A matrix
%   Blat: linearized state-space lateral B matrix
%   Clat: linearized state-space lateral C matrix
%   Dlat: linearized state-space lateral D matrix
%   XU0:  trim vector (order required by GUAM/GVS)
% OTHER UTILIZED FUNCTIONS:
%   poly_aero_wrapper_Mod_du.m: Wrapper to interogate polynomial model for 
%         forces/moments and derivatives 
%   lpc_aero_wrap.m: Wrapper to interogate strip theory model for 
%         forces/moments and derivatives
% *************************************************************************

% VERSION HISTORY
% 7.16.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS
% 
% 7.20.2023, Michael J. Acheson (NASA LaRC D-316): Updated version to
% include documentation and added polynomial model capability

global POLY

% gravity and density
rho  = 0.0023769;

% define some unit vectors, rotations, and skew symmetric matrix functions
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

Rx = @(x)  [1 0 0 ; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
Ry = @(x)  [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
Rz = @(x)  [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1];
hat = @(x) [ 0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

% Load in the equilibrium state
ubar   = XEQ(1);
wbar   = XEQ(2);
Radius = XEQ(3);
th     = XEQ(4);
phi    = XEQ(5);
psi    = 0;
p      = XEQ(6); 
q      = XEQ(7); 
r      = XEQ(8); 

% Load in equalibrium controls
surf_idx = 1:NS;
prop_idx = NS+1:NS+NP;
del  = UEQ(surf_idx);
omp  = UEQ(prop_idx);

% convert Radius to turn rate 
dpsi = ubar/Radius;

% body accelerations
ab = [ -g*sin(th); g*cos(th)*sin(phi); g*cos(th)*cos(phi) ];

eta = [ phi; th; psi];
vbar = [ubar; 0; wbar];
vb   = Rx(phi)*Ry(th)*vbar;
om = [ p; q; r ];

% save the trim state 
XU0 = [vbar; om; ab; eta; del; omp];
if POLY
    % Use Polynomial aero-database
    [F, M, F_x, M_x, F_u, M_u] = poly_aero_wrapper_Mod_du([vb; om]', [del; omp]', rho);
    % parse out the forces and moment and their derivatives    
    F_vb = F_x(1:3,1:3);
    M_vb = M_x(1:3,1:3);
    F_om = F_x(1:3,4:6);
    M_om = M_x(1:3,4:6);
    
    F_u = F_u(1:3,:);
    M_u = M_u(1:3,:);
else
    % Use strip-theory s-function approach
    [FM FM_x FM_u] = lpc_aero_wrap(tiltwing, [vb; om], [del; omp], rho);
    % parse out the forces and moment and their derivatives
    F = FM(1:3);
    M = FM(4:6);
    
    F_vb = FM_x(1:3,1:3);
    M_vb = FM_x(4:6,1:3);
    F_om = FM_x(1:3,4:6);
    M_om = FM_x(4:6,4:6);
    
    F_u = FM_u(1:3,:);
    M_u = FM_u(4:6,:);
end

% Pull out mass and inertia properties 
m  = tiltwing.mass;
J  = tiltwing.I;

S     = [1   sin(phi)*tan(th)   cos(phi)*tan(th) ; 
         0   cos(phi)          -sin(phi)         ; 
         0   sin(phi)/cos(th)   cos(phi)/cos(th)];

S_phi = [0   cos(phi)*tan(th)  -sin(phi)*tan(th) ; 
         0  -sin(phi)          -cos(phi)         ; 
         0   cos(phi)/cos(th)  -sin(phi)/cos(th)];

S_th  = [0   sin(phi)*sec(th)^2        cos(phi)*sec(th)^2       ; 
         0   0                         0                        ; 
         0   sin(phi)*tan(th)/cos(th)  cos(phi)*tan(th)/cos(th)];

Som_eta = [S_phi*om S_th*om zeros(3,1)];

% take some intermediate derivatives
Rbar     =  Rx(phi)*Ry(th);
Rbar_phi = -Rx(phi)*hat(e1)*Ry(th);
Rbar_th  = -Rx(phi)*Ry(th)*hat(e2);
Rbar_psi =  zeros(3,3);

vb_vbar = Rbar;
vb_eta = [Rbar_phi*vbar Rbar_th*vbar Rbar_psi*vbar];

% Derivatives of Forces and Moment with respect to states and inputs
Fbar_vbar = Rbar'*F_vb*vb_vbar;
Fbar_om   = Rbar'*F_om;
Fbar_eta  = [Rbar_phi'*F Rbar_th'*F Rbar_psi'*F] + Rbar'*F_vb*vb_eta;
Fbar_u    = Rbar'*F_u;

M_vbar = M_vb*vb_vbar;
M_eta  = M_vb*vb_eta;

% Build the state space matrices
A = [zeros(3,3)  Rz(psi)*hat(e3)  Rz(psi)                  zeros(3,3)                     ;
     zeros(3,3)  Som_eta          zeros(3,3)               S                              ;
     zeros(3,3)  1/m*Fbar_eta    -hat([0; 0; dpsi])+1/m*Fbar_vbar  1/m*Fbar_om            ;
     zeros(3,3)  J\M_eta          J\M_vbar                 J\(-hat(om)*J+hat(J*om)+M_om) ];

B = [ zeros(3,NS+NP) ; 
      zeros(3,NS+NP) ;
      1/m*Fbar_u     ;
      J\M_u      ];

C = eye(12);
D = zeros(12,NS+NP);

% SYS = ss(Alon, Blon, Clon, Dlon);