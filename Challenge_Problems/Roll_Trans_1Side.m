function [Px, Py, psi_delta] = Roll_Trans_1Side(grav, Vtotal, phi_start, phi_end, phi_time, psi_start) 

% This function is used to aid in generation of transition turns. 
% In particular, this script computes the change in postions (Px, Py)
% and/or the associated total heading change (psi).  This function aids in
% creating transition piecewise Bernstein polynomial waypoints
% (specifically it computes the change in position during a change of
% angle-of-bank and associated centripetal acceleration).  Examples of
% supported transitions include:
% 1) a straight line to a steady state turn or 
% 2) a steady state turn to to straight line segment or
% 3) a steady state turn to decreased turn rate turn
%
% Inputs:
% grav:             Gravity constant (units either in m/s^2 or ft/sec^2)
% V_total:          Scalar value (i.e., constant) or function handle with velocity
%                   function of time: V(t)
% phi_start:        angle-of-bank starting angle
% phi_end:          angle-of-bank ending angle
% phi_time:         time allowed to transition from starting to ending phi
% psi_start:        heading angle at start of maneuver
%
% Outputs: 
% Px:               Total distance traveled in inertial x-direction during maneuver
% Py:               Total distance traveled in inertial y-direction during maneuver
% psi_delta:        Total heading change during maneuver

% Written by Mike Acheson, NASA Langley Research Center
% Dynamic Systems and Control Branch (D-316)
% michael.j.acheson@nasa.gov

% Modification History:
% 4/4/2024 (MJA): original function written.

% *************************************************************************

% Set up the required integral functions
fn_omega    = @(s) grav/Vtotal*tan(-2*(phi_end-phi_start)/phi_time^3*s.^3+3*(phi_end-phi_start)/phi_time^2*s.^2+phi_start);
fn_Px   = @(x) Vtotal*cos(integral(fn_omega,0,x, 'ArrayValued', true)+psi_start);
fn_Py   = @(x) Vtotal*sin(integral(fn_omega,0,x, 'ArrayValued', true)+psi_start);

% Integrate the functions for the desired positions
Px = integral(fn_Px, 0, phi_time, 'ArrayValued', true);
Py = integral(fn_Py, 0, phi_time, 'ArrayValued', true);

% Integrate omega (w) for phi_time to determine total heading change
psi_delta = integral(fn_omega, 0, phi_time, 'ArrayValued', true);

% **********************  Equations used **********************************
% ******* Phi transition represented as a third order polynomial **********
% phi   =-2*(phi_end-phi_start)/phi_time^3*s.^3+3*(phi_end-phi_start)/phi_time^2*s.^2+phi_start
% Angular velocity (omega)  in turn: Acc/Vtotal
% instantaneous omega = g/Vtotal*tan(phi)
% delta_psi = integral(omega)
%
% Instantaneous Velocity:
% [Vx Vy]^T = Vtotal*[cos(phi); sin(phi)] (Inertial frame)
% Positions are integrals of Velocities
% Px = integral(Vtotal*cos(phi)), Py = integral(Vtotal*sin(phi)),