function [C]=LpC_Hover_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_GlobalModel_L - Blend partitioned aerodynamic model (L+C hover)
%
% DESCRIPTION: 
%   This function blends local aerodynamic models for the L+C hover regime
%   to create a global model.
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8]
%       Units: deg for angles, kts for velocity, rpm for rotor speed
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
%       Units: lbf for forces, ft-lbf for moments
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
%   16 MAR 2021 - created and debugged, BMS
%

%
%   Factor ranges used to develop model:
%             u: [      -5,     +45] kts
%             v: [     -10,     +10] kts
%             w: [     -10,     +10] kts
%            LA: [     -30,     +30] deg
%            RA: [     -30,     +30] deg
%            LE: [     -30,     +30] deg
%            RE: [     -30,     +30] deg
%           RUD: [     -30,     +30] deg
%            N1: [    +550,   +1550] rpm
%            N2: [    +550,   +1550] rpm
%            N3: [    +550,   +1550] rpm
%            N4: [    +550,   +1550] rpm
%            N5: [    +550,   +1550] rpm
%            N6: [    +550,   +1550] rpm
%            N7: [    +550,   +1550] rpm
%            N8: [    +550,   +1550] rpm
%

% initialize variables for Simulink implementation
UpperBound=0;
LowerBound=0;
C=zeros(1,8);
C1=zeros(1,8);
C2=zeros(1,8);
Blend=0;

% blend local models
switch true
   case u<(8 + 1/(-2))
      [C]=LpC_Hover_R7t10_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend=0; % no model blending is required
   case u>=(8 + 1/(-2)) && u<(20)
      [C1]=LpC_Hover_R7t10_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      [C2]=LpC_Hover_R7t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend = 1; % model blending is required
      UpperBound = 20; % upper limit of modeling interval
      LowerBound = 8 + 1/(-2); % lower limit of modeling interval
   case u>=(20) && u<(33 + 1/(-2))
      [C1]=LpC_Hover_R7t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      [C2]=LpC_Hover_R12t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend = 1; % model blending is required
      UpperBound = 33 + 1/(-2); % upper limit of modeling interval
      LowerBound = 20; % lower limit of modeling interval
   case u>=(33 + 1/(-2))
      [C]=LpC_Hover_R12t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend=0; % no model blending is required
end

if Blend==1
   % smooth blending between two regions using a quintic polynomial
   % weighting function, which is continuous and twice differentiable at
   % boundaries (Credit: Dan Moerder, NASA)
   wx= (UpperBound-u)/(UpperBound-LowerBound);
   w1=6*wx.^5-15*wx.^4+10*wx.^3;
   w2=1-w1;

   % combine weighted results
   C = w1*C1 + w2*C2;
end

return