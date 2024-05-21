function [Txb,Tyb,Tzb,Qxb,Qyb,Qzb]=LpC_Total_PropFM(u_fps,v_fps,w_fps,p_rps,q_rps,r_rps,n_revps,rho,a,Model)
% LpC_Total_PropFM - add forces and moments from each propulsor
%
% DESCRIPTION: 
%   This function add forces and moments from each propulsor.
% 
% INPUTS:
%   u_fps - x body-axis velocity [ft/s]
%   v_fps - y body-axis velocity [ft/s]
%   w_fps - z body-axis velocity [ft/s]
%   p_rps - roll rate [rad/s]
%   q_rps - pitch rate [rad/s]
%   r_rps - yaw rate [rad/s]
%   n_revps - Npx1 propeller speed [rev/s]
%   rho - air density [slug/ft^3]
%   a - speed of sound [ft/s]
%   Model - model parameters
%
% OUTPUTS:
%   Txb,Tyb,Tzb - body-axis force components [lbf]
%   Qxb,Qyb,Qzb - body-axis moment components (at the model reference 
%                 position) [ft-lbf]
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   Flight Dynamics Branch (D317)
%   NASA Langley Research Center
%
% HISTORY:
%   23 FEB 2021 - created and debugged, BMS
%

% degrees/radians conversion factors
d2r=pi/180;
r2d=180/pi;
np=length(Model.Prop_D); % number of propellers

% initialize body axis thrust variables
Txb=0;
Tyb=0;
Tzb=0;
Qxb=0;
Qyb=0;
Qzb=0;

% calculate true airspeed [ft/s]
V_fps=sqrt(u_fps.^2 + v_fps.^2 + w_fps.^2);

% motor locations
motor_loc=Model.Prop_location;

% initialize propeller incidence angle and flow direction parameters
i_deg=zeros(np,1);
Prop_flow_dir_deg=zeros(np,1);

% initialize local body velocity
up_b=zeros(np,1);
vp_b=zeros(np,1);
wp_b=zeros(np,1);

% CG location used to develop the aerodynamic models, this is NOT the
% current CG location in the simulation
modeling_cg_location=Model.cm_b;

% calculate the total propeller incidence angle [deg] and the local
% flow direction in the propeller disk plane [deg]
for kk=1:np
  R_HB=(Model.Prop_R_BH(1:3,1:3,kk))';
  [i_deg(kk),Prop_flow_dir_deg(kk),up_b(kk),vp_b(kk),wp_b(kk)]=...
      LpC_prop_inflow_angle(u_fps,v_fps,w_fps,p_rps,q_rps,r_rps,R_HB,modeling_cg_location,motor_loc(:,kk)');
end

V_fps_local=sqrt(up_b.^2+vp_b.^2+wp_b.^2);

% calculate the total advance ratio
J=V_fps_local./(n_revps.*Model.Prop_D'); % advance ratio

% calculate the propeller tip speed
V_tip_fps=2*pi*n_revps.*Model.Prop_D'/2; % tip velocity [ft/s]

% calculate the propeller tip mach number
Mtip=V_tip_fps/a; % tip velocity [ft/s]

% compute the propeller forces and moments from the local flow
% conditions at each individual propeller.
[Txbk,Tybk,Tzbk,Qxbk,Qybk,Qzbk,Tpf,Qpf]=LpC_Iso_PropFM(J,Mtip,n_revps,i_deg,Prop_flow_dir_deg,rho,modeling_cg_location,motor_loc,Model);

% body-axis thrust and torque components 
Txb=sum(Txbk,2);
Tyb=sum(Tybk,2);
Tzb=sum(Tzbk,2);
Qxb=sum(Qxbk,2);
Qyb=sum(Qybk,2);
Qzb=sum(Qzbk,2);



end
