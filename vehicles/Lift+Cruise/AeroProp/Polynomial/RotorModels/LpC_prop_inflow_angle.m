function [i_deg,Prop_flow_dir_deg,up_b,vp_b,wp_b]=LpC_prop_inflow_angle(u_fps,v_fps,w_fps,p_rps,q_rps,r_rps,R_HB,current_cg_location,propeller_location)
% LpC_prop_inflow_angle - calculate the propeller air-relative angles
%
% DESCRIPTION: 
%   This function calculates the propeller incidence angle and velocity
%   projection angle on the propeller disk plane. These propeller
%   orientation parameters are calculated from the body-axis velocity
%   components, angular rates, wing angle, and the propeller locations
%   relative to the CG.
%
% INPUT: 
%   u_fps - x body axis velocity [ft/s]
%   v_fps - y body axis velocity [ft/s]
%   w_fps - z body axis velocity [ft/s]
%   p_rps - roll rate [rad/s]
%   q_rps - pitch rate [rad/s]
%   r_rps - yaw rate [rad/s]
%   R_hb - rotation matrix from the body frame to the rotor hub frame
%   current_cg_location - 3x1 vector containing the current CG location [ft]
%   propeller_location - 3x1 vector containing the propeller location [ft]
%
% OUTPUT:
%   i_deg - propeller incidence angle [deg]
%   Prop_flow_dir - local propeller velocity projection angle onto the
%      propeller disk plane [deg]
%   up_b, vp_b, wp_b - local velocity components in the body frame [ft/s]
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   Flight Dynamics Branch (D317)
%   NASA Langley Research Center
%
% HISTORY:
%   23 FEB 2021 - Created and debugged, BMS
%

% freestream airspeed [ft/s]
V=sqrt(u_fps.^2+v_fps.^2+w_fps.^2);

% prevent the flow velocity from reaching zero
if abs(V)<0.0001
    u_fps=0.0001;
end

% distance from the propeller location to the aircraft CG [ft]
dx_p=propeller_location(1)-current_cg_location(1);
dy_p=propeller_location(2)-current_cg_location(2);
dz_p=propeller_location(3)-current_cg_location(3);

% velocity of the propeller in the body-fixed frame (including angular
% rates) [ft/s]
up_b = u_fps + q_rps*dz_p - r_rps*dy_p;
vp_b = v_fps + r_rps*dx_p - p_rps*dz_p;
wp_b = w_fps + p_rps*dy_p - q_rps*dx_p;

% rotate the local velocity to the propeller reference frame
Vp3=R_HB*[up_b;vp_b;wp_b];
up=Vp3(1);vp=Vp3(2);wp=Vp3(3);% propeller velocity components [ft/s]
Vp=sqrt(up.^2+vp.^2+wp.^2);

% calculate the propeller incidence angle, or propeller "total angle of
% attack" [deg]
i_deg=acosd(up/Vp);

% angle associated with the projection of the local flow velocity on the
% propeller disk plane
Prop_flow_dir_deg=atan2d(vp,wp);

return