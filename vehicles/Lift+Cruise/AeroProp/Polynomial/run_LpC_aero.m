function [Fb,Mb,Validity] = run_LpC_aero(x,n,d,rho,a,Units,Model)
% run_LpC_aero - run the Lift+Cruise polynomial aerodynamic models
%
% DESCRIPTION: 
%   This function runs the polynomial aerodynamic models for Lift+Cruise.
%   The function is called by a MATLAB function block in AeroPropRef.slx
%   (AeroPropRef/Propulsion and Aerodynamic Forces and Moments/Polynomial/
%   L+C Polynomial Model/L+C Aerodynamic Model)
%
% INPUTS: 
%   x - state vector containing body-axis velocity and body-axis angular 
%       rates, x=[u,v,w,p,q,r] in ft/s and rad/s
%   n - vector containing propeller speed values, n=[n1,n2...] in rad/s
%   d - vector contol surface positions, n=[d1,d2...] in rad
%   rho - atmosphereic density in slug/ft^3
%   a - speed of seound in ft/s
%   Units - SimIn.Units
%   Model - SimIn.Model
%
% OUTPUTS:
%   Fb - body-axis aerodynamic forces in lbf
%   Mb - body-axis aerodynamic moments in ft-lbf
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
%   04 JAN 2021 - Created and debugged, BMS
%

Fb=zeros(3,1);Mb=zeros(3,1); 

% select blending method for combining overlapping models
% 1: linear blending
% 2: smooth "sigmoid-like" blending (continuous and differentiable)
% 3: smooth quintic transition polynomial blending (continuous and twice differentiable)
blending_method=2;

% L+C version 2 aerodynamic models
[X,Y,Z,L,M,N,Validity] = LpC_aero_p_v2(x,n,d,rho,a,blending_method,Units,Model);

% body-axis forces [lbf]
Fb=[X;Y;Z];

% body-axis moments [ft-lbf]
Mb=[L;M;N];

end

