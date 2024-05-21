function [Alat, Blat, Clat, Dlat, XU0] = get_lat_dynamics_heading(tiltwing, XEQ, UEQ, NS, NP, rho, g, FreeVar_pnt, Trans_pnt)
% *************************************************************************
% function [Alat, Blat, Clat, Dlat, XU0] = get_lat_dynamics_heading(aircraft, XEQ, UEQ, NS, NP, rho, g)
%
% This script takes input of an aero/propulsive model (aircraft) either strip theory or
% polynomial, particular trim condition states (XEQ) and effectors (UEQ), 
% the number of aero surfaces (NS) and number of rotor/propeller surfaces (NP), and some
% Earth constants and outputs a linearized state-space lateral model 
% (expressed in the control frame specified in Ref 1.)
% 
%  dx = A*x + B*u
%   y = C*x + D*u
%
%   Desired linearized lateral output model
%   x = [ vbar p r phi ]'
%   u = [ omp1-8 dela delr ]'
%   y = [ vbar p r phi ]'
%
%  Definitions:
%     vb - velocity in the heading frame y direction
%      p - roll rate 
%      r - yaw rate 
%    phi - roll angle
%
% omp1-8 - lifting rotors
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
%   UEQ: Trim condition effector vector, order: 
%   NS: scalar number of aerodynamic surfaces
%   NP: scalar number of rotor/prop effectors 
%   rho: scalar value of Earth air density (slugs/ft^3)
%   grav: scalar value of Earth gravity acceleration (ft/sec^2)
%   FreeVar_pnt: Arrays of variables (including effectors) avail for control use
%   Trans_pnt: Column vector of transition start velocity and transition end velocity
% OUTPUTS:
%   Alat: linearized state-space lateral A matrix
%   Blat: linearized state-space lateral B matrix
%   Clat: linearized state-space lateral C matrix
%   Dlat: linearized state-space lateral D matrix
%   XU0:  trim vector (order required by GUAM/GVS)
% OTHER UTILIZED FUNCTIONS:
%   get_lin_dynamics_heading.m: % Get the linearized full state-space dynamics of vehicle 
% *************************************************************************

% VERSION HISTORY
% 7.16.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS
% 
% 7.20.2023, Michael J. Acheson (NASA LaRC D-316): Updated version to
% include documentation, changes to output lateral state-space
% matrices for desired (active) effectors

% Get the full linearized state-space representation
[A, B, C, D, XU0] = get_lin_dynamics_heading(tiltwing, XEQ, UEQ, NS, NP, rho, g);
% Determine what effectors are active in the trim maps
% FreeVar format: th phi p  q  r flp ail elv rud rl rt  pp
% Lin Dynamics formats: x = [ px py pz phi th psi ubar vbar wbar p q r ]'
%   u = [ delf dela dele delr omp1-9]'

% Convert_B_idx = [1 2 3 4 5:12 13];
% Convert_F_idx = Convert_B_idx+5;

% if XEQ(1) < Trans_pnt(1)
%     % Hover regime: Zero out B matrix col if FreeVar == zero
%     for chk = 1:length(Convert_B_idx)
%         if ~FreeVar_pnt(1,Convert_F_idx(chk))
%             B(:,Convert_B_idx(chk))= zeros(12,1);
%         end
%     end
% elseif XEQ(1) >=Trans_pnt(1) && XEQ(1) < Trans_pnt(2)
%     % Transition regime: Zero out B matrix col if FreeVar == zero
%     for chk = 1:length(Convert_B_idx)
%         if ~FreeVar_pnt(2,Convert_F_idx(chk))
%             B(:,Convert_B_idx(chk))= zeros(12,1);
%         end
%     end
% elseif XEQ(1) >= Trans_pnt(2)
%     % Cruise regime: Zero out B matrix col if FreeVar == zero
%     for chk = 1:length(Convert_B_idx)
%         if ~FreeVar_pnt(3,Convert_F_idx(chk))
%             B(:,Convert_B_idx(chk))= zeros(12,1);
%         end
%     end
% end

% get_lin_dynamics_heading states and controls
%   x = [ px py pz phi th psi ubar vbar wbar p q r ]'
%   u = [ delf dela dele delr omp1-9]'

% Specify desired state and control indices of linear dynamics output models
x_idx = [8 10 12 4]; % [vbar p r ph]
u_idx = [5:12 2 4]; % [omp1-8 ail rud] order matches what is in GUAM simulation
y_idx = x_idx; 

Alat = A(x_idx,x_idx);
Blat = B(x_idx,u_idx);
Clat = C(y_idx,x_idx);
Dlat = D(y_idx,u_idx);