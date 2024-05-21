function [Model]=LpC_model_parameters(SimIn)
% LpC_model_parameters - model parameters for the Lift+Cruise aircraft
%
% INPUTS:
%   SimIn
%
% OUTPUTS:
%   Model - model parameters for the polynomial model

% Modifications:
% 3-22-2023 MJA: Updated file to use SACD Lift+Cruise reference
% configuration (publicly releaseable configuration data)
% Lift+Cruise configuration information is from:
% https://sacd.larc.nasa.gov/uam-refs/

% See Lift+Cruise configuration, NDARC model files for Turbo-electric variant
% Ref. [1} list-cruiseTE6.list
% Ref. [2] lift-cruiseTE6.xlsv

Model.mass= 181.789249; % Computed using build_Lift_plus_Cruise
% Model.mass= 164.057876; % total vehicle mass [slug], see Ref. [2]

% total vehicle moment of inertia tensor [slug-ft^2]
Model.I = diag([13051.74318, 16660.75897, 24735.13582]);  % Computed using build_Lift_plus_Cruise

% Model.I = diag([9954.76683, 9524.15468, 17986.287957]); % Ixx, Iyy, Izz see Ref. [2]
% Ixy = 4.759894, Ixz=1072.91327, Iyz=2.886055 see RE[2]

Model.cm_b = [-13.841959237973750, -0.000000144320024, -4.603304281304035]; % Computed using build_Lift_plus_Cruise
%Model.cm_b=[-12.265719, 0.005163, -5.930782]; % vehicle CG location (+fwd, +right, +down) [ft] see Ref. [2]

% reference area [ft^2], Ref. [2]
Model.S= 186.0; % see Ref. [1]

% mean aerodynamic chord [ft], see Ref. [1]
Model.cbar=3.18;

% wingspan [ft], see Ref. [1]
Model.b=47.5;

% propeller/rotor locations [ft], see Ref. [2]
Model.Prop_location = ...
      [ -5.07,  -4.63, -4.63, -5.07,  -19.2,  -18.76,-18.76,-19.2,  -31.94;
       -18.750, -8.45,  8.45, 18.750, -18.750, -8.45,  8.45, 18.750,  0.000;
        -6.73,  -7.04, -7.04, -6.73,   -9.01,  -9.3,  -9.3,  -9.01,  -7.79];

% Propeller/rotor diameter [ft], see Ref [1]
Model.Prop_D=[10*ones(1,8),9];

% Propeller/rotor rotation direction (CW=+1, CCW=-1), see Ref. [1]
Model.prop_spin = [ -1  1  -1  1  1 -1  1 -1  1];
% rotors - as viewed from below looking up
%     Poly model: N1 => +N, N2 => -N, N3 => +N, N4 => -N
%                 N5 => -N, N6 => +N, N7 => -N, N8 => +N
% pusher prop - as viewed from the rear
%     Poly model: N9 => +L


% Propeller/rotor rotational moment of inertia [slug-ft^2], Ref. [1]
Model.Ip=[13.486*ones(1,8),17.486];

% propeller/rotor orientation angles (roll-pitch-yaw) [deg], see Ref. [1]
Model.Prop_angles=[  0,   0,   0,   0,   0,   0,   0,   0,   0
                   -90, -90, -90, -90, -90, -90, -90, -90,   0
                     0,  +8,  -8,   0,   0,  +8,  -8,   0,   0];

% unit vector of the axis of rotation and hub rotation matrix
Model.Prop_rot_axis_e=zeros(3,SimIn.numEngines);
Model.Prop_R_BH=zeros(3,3,SimIn.numEngines);
for ii=1:SimIn.numEngines
    % orientation angles [rad]
    phi=Model.Prop_angles(1,ii)*pi/180;
    theta=Model.Prop_angles(2,ii)*pi/180;
    psi=Model.Prop_angles(3,ii)*pi/180;
    
    % Rotation matrix from the body frame to the hub frame
    R_HB=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
           cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
                   -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)];
    
    % propeller/rotor axis of rotation matrix from the hub frame to the body frame       
    Model.Prop_R_BH(:,:,ii)=R_HB';
    
    % unit vector of the propeller/rotor axis of rotation         
    Model.Prop_rot_axis_e(:,ii)=R_HB'*[1;0;0];
end

% axial thrust direction unit vector (to match S-function calculations)
Model.p_T_e=Model.Prop_rot_axis_e;


% Diagrams from "setupControl.m"
%
% propellers are number from left to right, then front to back
% 
%                 
%                          / \
%                  (1) (2) | | (3) (4)
%                 ,-------------------,
%                 '-------------------'
%                  (5) (6) | | (7) (8)
%                          | |
%                       ,-------,
%                       '-------'
%                          (9)
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% surface allocation
%                            
%                          / \
%                          | |  
%                 ,-------------------,
%                 '-1---------------2-'
%                          | |  
%                          | |
%                       ,---|---,
%                       '3--|--4'
%                           5 
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    