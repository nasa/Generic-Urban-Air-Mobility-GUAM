function [AeroModel, Validity] = LpC_interp_p_v2(V_fps,rho,a,u,v,w,Alpha,Beta,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9,blending_method,Model)
% LpC_interp_p_v2 - This script interpolates between Lift+Cruise aero models
%
% DESCRIPTION: 
%   This script selects aerodynamic models identified for the Lift+Cruise
%   UAM-class vehicle based on the u velocity component. Models are blended
%   together in ranges of u which overlap.
%
% INPUTS: 
%   V_fps - freestream velocity [ft/s]
%   rho - atmospheric density [slug/ft^3]
%   a - speed of sound [ft/s]
%   u - x body-axis velocity [kts]
%   v - y body-axis velocity [kts]
%   w - z body-axis velocity [kts]
%   Beta - angle of sideslip [deg]
%   Alpha - angle of attack [deg]
%   LA - Left Aileron [deg]
%   RA - Right Aileron [deg]
%   LE - Left Elevator [deg]
%   RE - Right Elevator [deg]
%   RUD - Rudder [deg]
%   N1 - Engine 1 Speed [RPM]
%   N2 - Engine 2 Speed [RPM]
%   N3 - Engine 3 Speed [RPM]
%   N4 - Engine 4 Speed [RPM]
%   N5 - Engine 5 Speed [RPM]
%   N6 - Engine 6 Speed [RPM]
%   N7 - Engine 7 Speed [RPM]
%   N8 - Engine 8 Speed [RPM]
%   N9 - Engine 9 Speed [RPM]
%   blending_method - method of combining overlapping models
%         1=linear
%         2=smooth "sigmoid-like" blending (continuous and differentiable)
%         3=smooth "sigmoid-like" blending (continuous and twice differentiable)
%   Model - model parameters
%
% OUTPUTS:
%   AeroModel - matrix of aerodynamic forces [lbf] and moments [ft-lbf]
%               with column order: [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
%   Validity - 
%     invalid_speed - flag indicating invalid speed
%     invalid_prop_speed - flag indicating invalid prop speeds
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
%   04 MAR 2021 - created and debugged, BMS
%

% initialize return variable
AeroModel = zeros(1,6);

invalid_prop_speed = true;

invalid_speed = true;

% check for valid blending method
invalid_blending = false;
if (blending_method ~= 1) && (blending_method ~= 2) && (blending_method ~= 3)
  invalid_blending = true;
end

% calculate dynamic pressure [psf]
qbar=1/2*rho*V_fps^2;

% Lift+Cruise geometric properties
%   Note: cbar/2 and b/2 were used to calculate the force and moment
%   coefficients in the CFD results.
% S=231.49; cbar_half=1.97; b_half=23.86;
S=Model.S;% L+C wing area, ft^2
cbar_half=Model.cbar/2;% L+C half mean aerodynamic chord, ft^2
b_half=Model.b/2;% L+C half wing span, ft

% Use "hover" models (rotors on/propeller off)
if u>=-5*1.02 && u<=45*1.02 && min([N1;N2;N3;N4;N5;N6;N7;N8])>=550 && N9==0
    invalid_speed = false;invalid_prop_speed = false;
    
    u=check_fac_limits(u, -5, 45, 2.0);
    v=check_fac_limits(v, -10, 10, 2.0);
    w=check_fac_limits(w, -10, 10, 2.0);
    LA=check_fac_limits(LA, -30, 30, 2.0);
    RA=check_fac_limits(RA, -30, 30, 2.0);
    LE=check_fac_limits(LE, -30, 30, 2.0);
    RE=check_fac_limits(RE, -30, 30, 2.0);
    RUD=check_fac_limits(RUD, -30, 30, 2.0);
    N1=check_fac_limits(N1, 550, 1550, 2.0);
    N2=check_fac_limits(N2, 550, 1550, 2.0);
    N3=check_fac_limits(N3, 550, 1550, 2.0);
    N4=check_fac_limits(N4, 550, 1550, 2.0);
    N5=check_fac_limits(N5, 550, 1550, 2.0);
    N6=check_fac_limits(N6, 550, 1550, 2.0);
    N7=check_fac_limits(N7, 550, 1550, 2.0);
    N8=check_fac_limits(N8, 550, 1550, 2.0);
    
    % calculate the aerodynamic forces and moments using the L+C hover model
    C=LpC_Hover_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
    % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
    
    % aerodynamic forces and moments are the output from the model
    AeroModel=C(:,1:6);
    % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
    
% Use "transition" models (rotors on/propeller on)
elseif u>=-5*1.02 && u<=95*1.02 && min([N1;N2;N3;N4;N5;N6;N7;N8])>=550 && N9>=750
    invalid_speed = false;invalid_prop_speed = false;
    
    % Check factor limits - if model limits are exceeded hold the boundary value.
    u=check_fac_limits(u, -5, 95, 2.0);
    v=check_fac_limits(v, -10, 10, 2.0);
    w=check_fac_limits(w, -10, 10, 2.0);
    LA=check_fac_limits(LA, -30, 30, 2.0);
    RA=check_fac_limits(RA, -30, 30, 2.0);
    LE=check_fac_limits(LE, -30, 30, 2.0);
    RE=check_fac_limits(RE, -30, 30, 2.0);
    RUD=check_fac_limits(RUD, -30, 30, 2.0);
    N1=check_fac_limits(N1, 550, 1550, 2.0);
    N2=check_fac_limits(N2, 550, 1550, 2.0);
    N3=check_fac_limits(N3, 550, 1550, 2.0);
    N4=check_fac_limits(N4, 550, 1550, 2.0);
    N5=check_fac_limits(N5, 550, 1550, 2.0);
    N6=check_fac_limits(N6, 550, 1550, 2.0);
    N7=check_fac_limits(N7, 550, 1550, 2.0);
    N8=check_fac_limits(N8, 550, 1550, 2.0);
    N9=check_fac_limits(N9, 750, 1750, 2.0);
    
    % calculate the aerodynamic forces and moments using the L+C transition model
    C=LpC_Trans_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9);
    % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
    
    % aerodynamic forces and moments are the output from the model
    AeroModel=C(:,1:6);
    % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
    
    
% Use "transition" models (rotors on/propeller on) BUT with the propeller
% speed lower than its minimum value, and an airspeed greater than u=45 kts
% (low propeller speed operation below 45 kts is covered by the blending of
% hover and transition models below)
elseif u>45*0.98 && u<=95*1.02 && min([N1;N2;N3;N4;N5;N6;N7;N8])>=550 && N9>=0 && N9<750
    invalid_speed = false;invalid_prop_speed = false;
    
    % Check factor limits - if model limits are exceeded hold the boundary value.
    u=check_fac_limits(u, -5, 95, 2.0);
    v=check_fac_limits(v, -10, 10, 2.0);
    w=check_fac_limits(w, -10, 10, 2.0);
    LA=check_fac_limits(LA, -30, 30, 2.0);
    RA=check_fac_limits(RA, -30, 30, 2.0);
    LE=check_fac_limits(LE, -30, 30, 2.0);
    RE=check_fac_limits(RE, -30, 30, 2.0);
    RUD=check_fac_limits(RUD, -30, 30, 2.0);
    N1=check_fac_limits(N1, 550, 1550, 2.0);
    N2=check_fac_limits(N2, 550, 1550, 2.0);
    N3=check_fac_limits(N3, 550, 1550, 2.0);
    N4=check_fac_limits(N4, 550, 1550, 2.0);
    N5=check_fac_limits(N5, 550, 1550, 2.0);
    N6=check_fac_limits(N6, 550, 1550, 2.0);
    N7=check_fac_limits(N7, 550, 1550, 2.0);
    N8=check_fac_limits(N8, 550, 1550, 2.0);
    N9=check_fac_limits(N9, 0, 750, 0);
    
    % lower limit of pusher prop validity for the L+C transition model
    N9_trans_limit=750;
    
    % calculate the aerodynamic forces and moments using the L+C transition model
    C=LpC_Trans_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9_trans_limit);
    % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
    
    % calculate the isolated propulsion forces/moments for the
    % propulsor(s) being blended into the vehicle model
    N_rpm_value = [zeros(8,1);N9];% propulsor speed vector, RPM
    N_rpm_lim = [zeros(8,1);N9_trans_limit];% propulsor speed limit vector, RPM
    N_rpm_model_lim = N9_trans_limit;% propulsor speed limit
    AeroModel_p=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);

    % add the transition model (with N9 held to its boundary limit) to
    % the difference between the estimated propeller forces/moment and
    % the propeller forces/moments at the transition model limit.
    % This models lower RPM operation than the transition model allows.
    AeroModel=C(:,1:6)+AeroModel_p; 
    % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
 
% Use "cruise" models (rotors off/propeller on)
    %elseif u>=50*0.98 && u<=130*1.02 && max(abs([N1;N2;N3;N4;N5;N6;N7;N8]))==0 && N9>=550
elseif u>=50*0.98 && u<=130*1.02 && max(abs([N1;N2;N3;N4;N5;N6;N7;N8]))<=1 && N9>=550

    invalid_speed = false;invalid_prop_speed = false;
    
    % Check factor limits - if model limits are exceeded hold the boundary value.
    u=check_fac_limits(u, 50, 130, 2.0);
    Beta=check_fac_limits(Beta, -6, 6, 2.0);
    Alpha=check_fac_limits(Alpha, 0, 12, 2.0);
    LA=check_fac_limits(LA, -30, 30, 2.0);
    RA=check_fac_limits(RA, -30, 30, 2.0);
    LE=check_fac_limits(LE, -30, 30, 2.0);
    RE=check_fac_limits(RE, -30, 30, 2.0);
    RUD=check_fac_limits(RUD, -30, 30, 2.0);
    N9=check_fac_limits(N9, 550, 1750, 2.0);
    
    % calculate pseudo advance ratio [ (rev/s)/(ft/s) ]
    %   (note that this does NOT include the held boundary value of "V_fps"
    %    if the model limits are exceeded.)
    n9oV=(N9/60)/V_fps;
    
    % calculate the aerodynamic forces and moments using the L+C Cruise model
    C=LC_C_ab_Jh_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD,n9oV);
    % Output: C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]
    
    % calculate aerodynamic forces and moments from the force and moment
    % coefficients.
    AeroModel=qbar*S*[C(:,1:3), C(:,4)*b_half, C(:,5)*cbar_half, C(:,6)*b_half];
    % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
    
% Use "glider" models (rotors off/propeller off)
elseif u>=50*0.98 && u<=130*1.02 && max(abs([N1;N2;N3;N4;N5;N6;N7;N8]))==0 && N9==0
    invalid_speed = false;invalid_prop_speed = false;
    
    % Check factor limits - if model limits are exceeded hold the boundary value.
    u=check_fac_limits(u, 50, 130, 2.0);
    Beta=check_fac_limits(Beta, -6, 6, 2.0);
    Alpha=check_fac_limits(Alpha, 0, 12, 2.0);
    LA=check_fac_limits(LA, -30, 30, 2.0);
    RA=check_fac_limits(RA, -30, 30, 2.0);
    LE=check_fac_limits(LE, -30, 30, 2.0);
    RE=check_fac_limits(RE, -30, 30, 2.0);
    RUD=check_fac_limits(RUD, -30, 30, 2.0);
    
    % calculate the aerodynamic forces and moments using the L+C Glider model
    C=LC_G_ab_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD);
    % Output: C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]
    
    % calculate aerodynamic forces and moments from the force and moment
    % coefficients.
    AeroModel=qbar*S*[C(:,1:3), C(:,4)*b_half, C(:,5)*cbar_half, C(:,6)*b_half];
    % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

else
% Blending between different flight regimes.
    
    % initialize variables
    Nlow_end = 1;
    Nhigh_start = 0;
    Nblend=0;
    AeroModel_1 = zeros(1,6);
    AeroModel_2 = zeros(1,6);
    AeroModel_p_1 = zeros(1,6);
    AeroModel_p_2 = zeros(1,6);
    AeroModel_rot_1 = zeros(1,6);
    AeroModel_rot_2 = zeros(1,6);
    

    if u>=-5*1.02 && u<=45*1.02 && min([N1;N2;N3;N4;N5;N6;N7;N8])>=550 && N9>0 && N9<750
    % blending between hover and transition (turning on/off the pusher
    % propeller at low speeds) 
    
        invalid_speed = false;
        invalid_prop_speed = false;
    
        % model 1 high rotor speed [rpm]
        Nlow_end=750;

        % model 2 low rotor speed [rpm]
        Nhigh_start=0;
        
        % mean rotor speed for blending [rpm]
        Nblend=N9;
            
        % Check factor limits - if model limits are exceeded hold the boundary value.
        u=check_fac_limits(u, -5, 45, 2.0);
        v=check_fac_limits(v, -10, 10, 2.0);
        w=check_fac_limits(w, -10, 10, 2.0);
        LA=check_fac_limits(LA, -30, 30, 2.0);
        RA=check_fac_limits(RA, -30, 30, 2.0);
        LE=check_fac_limits(LE, -30, 30, 2.0);
        RE=check_fac_limits(RE, -30, 30, 2.0);
        RUD=check_fac_limits(RUD, -30, 30, 2.0);
        N1=check_fac_limits(N1, 550, 1550, 2.0);
        N2=check_fac_limits(N2, 550, 1550, 2.0);
        N3=check_fac_limits(N3, 550, 1550, 2.0);
        N4=check_fac_limits(N4, 550, 1550, 2.0);
        N5=check_fac_limits(N5, 550, 1550, 2.0);
        N6=check_fac_limits(N6, 550, 1550, 2.0);
        N7=check_fac_limits(N7, 550, 1550, 2.0);
        N8=check_fac_limits(N8, 550, 1550, 2.0);
        N9=check_fac_limits(N9, 0, 750, 0);

        
    % run model 1 (upper factor range)
    
        % calculate the aerodynamic forces and moments using the L+C hover model
        C_1=LpC_Hover_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
        % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);0];% propulsor speed limit vector, RPM
        N_rpm_model_lim = 0;% propulsor speed limit
        AeroModel_p_1=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);
        
        % add the hover model to the pusher propeller contribution
        AeroModel_1= C_1(:,1:6) + AeroModel_p_1;

    % run model 2 (lower factor range)
    
        % lower limit of pusher prop validity for the L+C transition model
        N9_trans_limit=750;
        
        % calculate the aerodynamic forces and moments using the L+C
        % transition model with N9 held to its boundary limit
        C_2=LpC_Trans_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9_trans_limit);
        % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]

        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);N9_trans_limit];% propulsor speed limit vector, RPM
        N_rpm_model_lim = N9_trans_limit;% propulsor speed limit
        AeroModel_p_2=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);
        
        % add the transition model (with N9 held to its boundary limit) to
        % the difference between the estimated propeller forces/moment and
        % the propeller forces/moments at the transition model limits.
        % This models lower RPM operation than the transition model allows.
        AeroModel_2=C_2(:,1:6)+AeroModel_p_2; 
        
        
    %elseif u>=50*0.98 && u<=95*1.02 && max([N1;N2;N3;N4;N5;N6;N7;N8])>0
    elseif u>=50*0.98 && u<=100*1.02 && max([N1;N2;N3;N4;N5;N6;N7;N8])>0

    % blending between transition and cruise (turning on/off the lifting
    % rotors at speeds between 50 to 95 kts)
        invalid_speed = false;  
        invalid_prop_speed = false;
        
        % model 1 high rotor speed [rpm]
        Nlow_end=550;

        % model 2 low rotor speed [rpm]
        Nhigh_start=0;
        
        % mean rotor speed for blending [rpm]
        Nblend=min([mean([N1;N2;N3;N4;N5;N6;N7;N8]),Nlow_end]);
      
        % Check factor limits - if model limits are exceeded hold the boundary value.
        u=check_fac_limits(u, 50, 100, 2.0);
        v=check_fac_limits(v, -10, 10, 2.0);
        w=check_fac_limits(w, -10, 10, 2.0);
        Beta=check_fac_limits(Beta, -6, 6, 2.0);
        Alpha=check_fac_limits(Alpha, 0, 12, 2.0);
        LA=check_fac_limits(LA, -30, 30, 2.0);
        RA=check_fac_limits(RA, -30, 30, 2.0);
        LE=check_fac_limits(LE, -30, 30, 2.0);
        RE=check_fac_limits(RE, -30, 30, 2.0);
        RUD=check_fac_limits(RUD, -30, 30, 2.0);
        N1=check_fac_limits(N1, 0, 1550, 2.0);
        N2=check_fac_limits(N2, 0, 1550, 2.0);
        N3=check_fac_limits(N3, 0, 1550, 2.0);
        N4=check_fac_limits(N4, 0, 1550, 2.0);
        N5=check_fac_limits(N5, 0, 1550, 2.0);
        N6=check_fac_limits(N6, 0, 1550, 2.0);
        N7=check_fac_limits(N7, 0, 1550, 2.0);
        N8=check_fac_limits(N8, 0, 1550, 2.0);
        N9=check_fac_limits(N9, 0, 1750, 2.0);
        
     % run model 1 (upper factor range)
     
        % lower limit of pusher prop validity for the L+C cruise model
        N9_cruise_limit=550;
        [N9_PM,N9_RM,N9_RM_lim]=rotor_blending_limits(N9,N9_cruise_limit);
    
        % calculate pseudo advance ratio [ (rev/s)/(ft/s) ]
        % (note that this does NOT include the held boundary value of "V_fps"
        %  if the model limits are exceeded.)
        n9oV=(N9_PM/60)/V_fps;

        % calculate the aerodynamic forces and moments using the L+C Cruise model
        C_1=LC_C_ab_Jh_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD,n9oV);
        % Output: C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]

        % calculate aerodynamic forces and moments from the force and moment
        % coefficients.
        FM_1=qbar*S*[C_1(:,1:3), C_1(:,4)*b_half, C_1(:,5)*cbar_half, C_1(:,6)*b_half];
        % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [N1;N2;N3;N4;N5;N6;N7;N8;0];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);0];% propulsor speed limit vector, RPM
        N_rpm_model_lim = 0;% propulsor speed limit
        AeroModel_rot_1=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);

        % pitching moment appears to be over-predicted by the rotor models
        % compared to the polynomial model--exclude pitching moment from
        % the blending calculations
        AeroModel_rot_1(1,5)=0;
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9_RM];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);N9_RM_lim];% propulsor speed limit vector, RPM
        N_rpm_model_lim = N9_cruise_limit;% propulsor speed limit
        AeroModel_p_1=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);

        % add the cruise model to the rotor and prop contributions
        AeroModel_1= FM_1 + AeroModel_rot_1 + AeroModel_p_1;

    % run model 2 (lower factor range)
    
        % lower limit of rotor validity for the L+C transition model
        rotor_trans_limit=550;
        
        % calculate rotor values needed for calculations
        [N1_PM,N1_RM,N1_RM_lim]=rotor_blending_limits(N1,rotor_trans_limit);
        [N2_PM,N2_RM,N2_RM_lim]=rotor_blending_limits(N2,rotor_trans_limit);
        [N3_PM,N3_RM,N3_RM_lim]=rotor_blending_limits(N3,rotor_trans_limit);
        [N4_PM,N4_RM,N4_RM_lim]=rotor_blending_limits(N4,rotor_trans_limit);
        [N5_PM,N5_RM,N5_RM_lim]=rotor_blending_limits(N5,rotor_trans_limit);
        [N6_PM,N6_RM,N6_RM_lim]=rotor_blending_limits(N6,rotor_trans_limit);
        [N7_PM,N7_RM,N7_RM_lim]=rotor_blending_limits(N7,rotor_trans_limit);
        [N8_PM,N8_RM,N8_RM_lim]=rotor_blending_limits(N8,rotor_trans_limit);
        
        % lower limit of pusher prop validity for the L+C transition model
        N9_cruise_limit=750;
        [N9_PM,N9_RM,N9_RM_lim]=rotor_blending_limits(N9,N9_cruise_limit);
        
        % calculate the aerodynamic forces and moments using the L+C
        % transition model with rotors held to their boundary limits, if
        % needed
        C_2=LpC_Trans_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1_PM,N2_PM,N3_PM,N4_PM,N5_PM,N6_PM,N7_PM,N8_PM,N9_PM);
        % Output: C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
        

        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [N1_RM;N2_RM;N3_RM;N4_RM;N5_RM;N6_RM;N7_RM;N8_RM;0];% propulsor speed vector, RPM
        N_rpm_lim = [N1_RM_lim;N2_RM_lim;N3_RM_lim;N4_RM_lim;N5_RM_lim;N6_RM_lim;N7_RM_lim;N8_RM_lim;0];% propulsor speed limit vector, RPM
        N_rpm_model_lim = rotor_trans_limit;% propulsor speed limit
        AeroModel_rot_2=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);

        % pitching moment appears to be over-predicted by the rotor models
        % compared to the polynomial model--exclude pitching moment from
        % the blending calculations
        AeroModel_rot_2(1,5)=0;
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9_RM];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);N9_RM_lim];% propulsor speed limit vector, RPM
        N_rpm_model_lim = N9_cruise_limit;% propulsor speed limit
        AeroModel_p_2=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);

        
        % add the transition model (with N9 held to its boundary limit) to
        % the difference between the estimated propeller forces/moment and
        % the propeller forces/ moments at the transition model limits.
        % This models lower RPM operation than the transition model allows.
        AeroModel_2=C_2(:,1:6) + AeroModel_rot_2 + AeroModel_p_2;

    
    elseif u>=50*0.98 && u<=130*1.02 && min([N1;N2;N3;N4;N5;N6;N7;N8])==0 && N9>0 && N9<550
    % blending between the cruise and glider models (turning off the
    % pusher propeller in forward flight)
    
        invalid_speed = false;  
        invalid_prop_speed = false;
        
        % model 1 high rotor speed [rpm]
        Nlow_end=550;

        % model 2 low rotor speed [rpm]
        Nhigh_start=0;
        
        % mean rotor speed for blending [rpm]
        Nblend=N9;
    
        % Check factor limits - if model limits are exceeded hold the boundary value.
        u=check_fac_limits(u, 50, 130, 2.0);
        Beta=check_fac_limits(Beta, -6, 6, 2.0);
        Alpha=check_fac_limits(Alpha, 0, 12, 2.0);
        LA=check_fac_limits(LA, -30, 30, 2.0);
        RA=check_fac_limits(RA, -30, 30, 2.0);
        LE=check_fac_limits(LE, -30, 30, 2.0);
        RE=check_fac_limits(RE, -30, 30, 2.0);
        RUD=check_fac_limits(RUD, -30, 30, 2.0);
        N9=check_fac_limits(N9, 0, 550,0);    
        
   % run model 1 (upper factor range)
    
        % calculate the aerodynamic forces and moments using the L+C Glider model
        C_1=LC_G_ab_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD);
        % Output: C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]

        % calculate aerodynamic forces and moments from the force and moment
        % coefficients.
        FM_1=qbar*S*[C_1(:,1:3), C_1(:,4)*b_half, C_1(:,5)*cbar_half, C_1(:,6)*b_half];
        % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);0];% propulsor speed limit vector, RPM
        N_rpm_model_lim = 0;% propulsor speed limit
        AeroModel_p_1=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);
        
        % add the glider model to the pusher propeller contribution
        AeroModel_1= FM_1 + AeroModel_p_1;

    % run model 2 (lower factor range)
    
        % lower limit of pusher prop validity for the L+C cruise model
        N9_cruise_limit=550;
        
        % calculate pseudo advance ratio [ (rev/s)/(ft/s) ]
        %   (note that this does NOT include the held boundary value of "V_fps"
        %    if the model limits are exceeded.)
        n9oV=(N9_cruise_limit/60)/V_fps;

        % calculate the aerodynamic forces and moments using the L+C Cruise model
        C_2=LC_C_ab_Jh_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD,n9oV);
        % Output: C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]

        % calculate aerodynamic forces and moments from the force and moment
        % coefficients.
        FM_2=qbar*S*[C_2(:,1:3), C_2(:,4)*b_half, C_2(:,5)*cbar_half, C_2(:,6)*b_half];
        % AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
        
        % calculate the isolated propulsion forces/moments for the
        % propulsor(s) being blended into the vehicle model
        N_rpm_value = [zeros(8,1);N9];% propulsor speed vector, RPM
        N_rpm_lim = [zeros(8,1);N9_cruise_limit];% propulsor speed limit vector, RPM
        N_rpm_model_lim = N9_cruise_limit;% propulsor speed limit
        AeroModel_p_2=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model);
        
        % add the cruise model (with N9 held to its boundary limit) to
        % the difference between the estimated propeller forces/moment and
        % the propeller forces/moments at the cruise model limits.
        % This models lower RPM operation than the transition model allows.
        AeroModel_2=FM_2+AeroModel_p_2;
     
    end
    
    % difference between the lower model high end and upper model low end
    Delta_N = Nlow_end - Nhigh_start;
    % initialize default weighting values
    w1 = 0.0;
    w2 = 0.0;
    if (invalid_speed == false)
      if blending_method==1 % linear weighting
        % linear weights (continuous)
        w1=(Nlow_end-Nblend)/Delta_N;
        w2=1-w1;
      elseif blending_method==2 % smooth weighting
        % "sigmoid-like" function for smooth blending (continuous and once differentiable)
        w_angle= (Nlow_end-Nblend)/Delta_N * pi/2;
        w1=sin(w_angle).^2;
        w2=1-w1;
      elseif blending_method==3 % smooth weighting (continuous and twice differentiable)
        % "sigmoid-like" function for smooth blending (Credit: Dan Moerder)
        wx= (Nlow_end-Nblend)/Delta_N;
        w1=6*wx.^5-15*wx.^4+10*wx.^3;
        w2=1-w1;
      end
    end
    
    % check valiity of the weighting functions
    if w1<0 || w1>1 || w2<0 || w2>1 
        invalid_blending = true;
    end
    % combine weighted results
    AeroModel = w1*AeroModel_1 + w2*AeroModel_2;
    
end




Validity = [invalid_speed invalid_prop_speed];

%% error statements not needed for these 2 flags due to assertion checks on
%% vehicle model in simulink

%if (invalid_speed == true)
%  error('the aerodynamic model''s validity has been violated')
%end

%if (invalid_prop_speed == true)
%  error('rotor/prop speed is above and below the minimum operational speed')
%end

if (invalid_blending == true)
  error('invalid interpolation method')
end

% Output: AeroModel = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];
%         Validity  = [invalid_speed invalid_prop_speed];    
end


function AeroModel_p=blending_prop_calculations(N_rpm_value,N_rpm_lim,N_rpm_model_lim,u,v,w,rho,a,Model)
% blending_prop_calculations - compute the isolated propulsion forces/
%   moments needed for blending flight regimes
%
% INPUTS
%   N_rpm_value - 9x1 vector of the actual propulsor rotational speed values
%   N_rpm_lim - 9x1 vector of the propulsor rotational speed values at the
%               vehicle model limits
%   N_rpm_model_lim - vehicle model propulsor speed limit
%   u - x body-axis velocity [kts]
%   v - y body-axis velocity [kts]
%   w - z body-axis velocity [kts]
%   rho - atmospheric density [slug/ft^3]
%   a - speed of sound [ft/s]
%   Model - model parameters
% 
% OUTPUTS
%   AeroModel_p - propulsion forces and moments to add to the aerodynamic
%                 forces and moments
%

% calculate the estimated static forces and moments from the
% rotors/propeller, the value is set to zero if the polynomial model is
% within the limits for a propulsor
n_revps_value=N_rpm_value/60;
[Xp,Yp,Zp,Lp,Mp,Np]=LpC_Total_PropFM(u,v,w,0,0,0,n_revps_value,rho,a,Model);

if N_rpm_model_lim>0
% if the model limit for RPM is greater than zero, then subtract out
% propulsion contributions
    
    % calculate the estimated static forces and moments from the rotors or
    % propeller at the model limit
    n_revps_lim=N_rpm_lim/60;
    [Xp_lim,Yp_lim,Zp_lim,Lp_lim,Mp_lim,Np_lim]=LpC_Total_PropFM(u,v,w,0,0,0,n_revps_lim,rho,a,Model);
    
    % Computes the difference between the estimated rotors/propeller
    % forces/moments and the propeller forces/ moments at the vehicle model
    % limits. This is used to model lower RPM operation than the vehicle
    % model allows.
    AeroModel_p = [-Xp,Yp,-Zp,Lp,Mp,Np] - [-Xp_lim,Yp_lim,-Zp_lim,Lp_lim,Mp_lim,Np_lim];
    
else
% if the model limit for RPM is zero, then only add propulsion contributions
        
    AeroModel_p = [-Xp,Yp,-Zp,Lp,Mp,Np];
end


end



function [N_PM,N_RM,N_RM_lim]=rotor_blending_limits(N,N_Lower_Limit)
% rotor_blending_limits - calculate the rotor limits need for flight regime blending
%
% INPUTS
%   N - actual rotor speed
%   N_Lower_Limit - rotor speed lower limit
%
% OUPUTS
%   N_PM - rotor speed used in the polynomial model
%   N_RM - rotor speed used in the rotor model
%   N_RM_lim - rotor speed used in the rotor model lower limit value
%

if N < N_Lower_Limit
    % hold the rotor speed sent to the polynomial model to the lower limit
    % value
    N_PM = N_Lower_Limit;
    
    % the value sent to the rotor models will be the actual rotor speed and
    % the limiting value for the polynomial model
    N_RM = N;
    N_RM_lim = N_Lower_Limit;
    
else
    % the polynomial model will be used at its actual rotor speed value
    N_PM = N;
    
    % the rotor model will not be used, thus set the values to zero (the
    % effects are within the limits of the polynomial model)
    N_RM = 0;
    N_RM_lim = 0;
end

end