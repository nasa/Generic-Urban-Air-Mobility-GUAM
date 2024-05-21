function [CT,CQ]=LpC_PusherProp_Model(Jx)
% LpC_PusherProp_Model - Pusher propeller aerodynamic model for Lift+Cruise
%
% DESCRIPTION: 
%   This script contains the pusher propeller aerodynamics model for the
%   NASA Lift+Cruise aircraft. The modeling methodology for CT and CQ is
%   described below.
% 
% INPUTS:
%   Jx - normal advance ratio:
%        Jx=V*cosd(Alpha)*cosd(Beta)/(n*D)=u/(n*D)
%        where: 
%             V = airspeed [ft/s]
%             u = x body-axis velocity [ft/s]
%             Alpha = angle of attack [deg]
%             Beta = angle of sideslip [deg]
%             n = propeller rotational speed [rev/s]
%             D = propeller diameter [ft]
%
% OUTPUTS:
%   CT - thrust coefficient [CT=T/(rho*n^2*D^4)]
%   CQ - torque coefficient [CQ=Q/(rho*n^2*D^5)]
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   Flight Dynamics Branch (D317)
%   NASA Langley Research Center
%
% HISTORY:
%   23 FEB 2021 - Created and debugged, BMS
%

% thrust coefficient
CT = -2.123873237735236e-01.*Jx.^2 + 3.965946758735335e-01;

% torque coefficient
CQ = 3.705976385963508e-02.*Jx.^2 -6.599151230379886e-02;

end