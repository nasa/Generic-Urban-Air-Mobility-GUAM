function [tiltwing] = build_Lift_plus_Cruise()
% Tiltwing Aircraft Build Script
%

%
% This file is used to specify the tiltwing aircraft 
% configuration. It is called within the MATLAB S-Function
% tiltwing_aero_s.m. The S-Function is used to make an
% tiltwing aerodynamics block within simulink.
% 
% The script produces a "TiltwingClass" class object
% that stores all relevant aircraft information and
% provides a single function to compute the 
% aerodynamic forces and moments based on the
% current aircraft configuration.
%
% The aero function provides the forces and moment
% in the body frame from the velocity, angle of attack,
% side slip angle, and body anglur rates.
% 
% [Fb Mb] = tiltwing.aero(rho, vb, om)

% Modifications:
% 
% 7/20/2018, Jacob Cook: Original script.
% 
% 3-22-2023, MJA: Updated vehicle properties to utilize publicly releasable
% properties found in SACD Lift+Cruise reference configuration at
% https://sacd.larc.nasa.gov/uam-refs/, 
% See Lift+Cruise configuration, NDARC model files for Turbo-electric variant
% Ref. [1] list-cruiseTE6.list
% Ref. [2] lift-cruiseTE6.xlsv

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fuselage 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% See NDARC file: lift-cruiseTE6.list and lift-cruiseTE6.xlsx
% mass of fuselage
m_b = 15.294862; % See Ref. [2] cell B16
% moment of inertia of fuselage
Ib = diag([74.788921	460.34866	451.179883]); % See Ref. [2] cell F16-H16

% body center of mass in the body frame
body_cm_b = [-9; 0; -4.25]; % Ref. [1] line 1060
% cross sectional area
S_b = 42.07; % See Ref. [1] line 161
% planform area
S_p =  172.69; % planform area See Ref. [1] lines 157 and 158 (product of length and width)
% wetted area
S_s =368.851; % wetted area, See Ref. [1] line 160
% fineness factor, length/max diameter
f_ld = 29.93/6.13; % fineness factor, length/max diameter  See Ref[1] lines 157 & 158

%% Build the body %%
Body = BodyClass( m_b, ...
                  Ib, ...
                  body_cm_b, ...
                  S_b, ...
                  S_p, ...
                  S_s, ...
                  f_ld);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Propeller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% wing profile
wing_airfoil = load('naca633618.dat');
% wing aerodynamic coefficients
wing_aero_coeff = load('NACA_0015_pp.mat'); % loaded data is a structure already
% wing span and exposed wing span
b = 47.5;   % See Ref. [1] line 451
b_e = 47.5; % See Ref. [1] line 451
% chord length
c = [6.34513 3.000]; % Created using Ref. [1] wing definition (lines 443-479)
% dihedral
gamma = -2*pi/180; % Created using Ref. [1] wing definition (lines 443-479)
% mass
m_w = 12.044978; % See Ref. [2] cell B56
% moment of inertia
Iw = diag([1608.52513	12.672535	1618.725065]); % See Ref. [2] cell F56-H56
% wing center of mass in the body frame
w_cm_b = [-10.69; 0.000; -8.5]; % Ref. [1] line 1080
% wing quarter chord location in the body frame
c4_b = w_cm_b;

% flap location
y_flap = [0.45 0.95]*b/2; % Created for this configuration

% aileron location
y_aileron = [0.45*b/2 0.95*b/2]; % Created for this configuration

%% Build the Wing %%
Wing = WingClass( wing_airfoil, ...
                  wing_aero_coeff.NACA_0015_pp, ...
                  [b b_e], ...
                  c, ...
                  gamma,...
                  y_flap, ...
                  y_aileron, ...
                  c4_b, ...
                  m_w, ... 
                  Iw, ...
                  w_cm_b);

        
%% Build the wing-propeller combination %%
WingProp = WingPropClass(Wing);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tail
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Horizontal Tail %%

% wing profile
airfoil_t = load('n0012.dat');
% aerodynamic coefficients

% span and exposed span
b_ht      = 10.3328; % See Ref. [1], estimated as area (line 291) divided by mean cord (see below)
b_e_ht    = 10.3328;
% chord 
c_ht      = [3.00 1.80]; % Chosen for this application
% dihedral
gamma = 0.0; % See Ref. [1] line 494
% elevator location
y_flap    = [0.0 1.0]*b_ht/2; % Whole span elevator
% set the aileron to zero
y_aileron = [0 0]; % No aileron on HT
% h. tail center of mass in the body frame
ht_cm_b   = [ -29.1; 0.000; -8.01]; % Ref. [1] line 1081
% h. tail quarter chord location in the body frame
ht_c4_b   = ht_cm_b;
% mass of h. tail
m_ht      = 2.262387; % See Ref. [2] cell B57
% inertia matrix
Iht       = diag([3.473351 8.689178 5.338722]); % See Ref. [2] cells F57-H57

%% Build the horizontal tail %%
hTail = WingClass( airfoil_t, ...
                   wing_aero_coeff.NACA_0015_pp, ...
                   [b_ht b_e_ht], ...
                   c_ht, ...
                   gamma,...
                   y_flap, ...
                   y_aileron, ...
                   ht_c4_b, ...
                   m_ht, ...
                   Iht, ...
                   ht_cm_b); 

%% Vertical Tail %%

% span 
b_vt     = 4.97524; % Selected for this configuration
% chord
c_vt     = [8.5 2.32902]; % Selected for this configuration
% v. tail quarter chord location in the body frame
vt_c4_b  = [-28.35; 0.00; -10.3]; % see Ref. [1] line 1082
% mass of v. tail
m_vt     = 0.68167; % See Ref. [2] cell B58
% inertia matrix
Ivt      = diag([5.209589 0.228829 5.430544]); % See Ref[2] cells F58-H58
% v. tail center of mass in the body frame
vt_cm_b  = vt_c4_b;
% rudder location
y_rudder = [0 1]*b_vt;

%% Build the vertical tail %% 
vTail = VerticalTailClass( airfoil_t,...
                           wing_aero_coeff.NACA_0015_pp,...
                           b_vt, ...
                           c_vt, ...
                           y_rudder, ...
                           vt_c4_b, ...
                           m_vt, ...
                           Ivt, ...
                           vt_cm_b);

%% Build the tail with horizontal and Vertical components %%
Tail = TailClass(hTail, vTail);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lift and Cruise Propellers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of props
NP = 9; % See Ref. [1]

% diameter of each propellers. See Ref. [1] for each rotor radius...
D = [10.0 10.0 10.0 10.0 10.0 10.0 10.0 10.0 9.0];

% Note: NDARC has different rotor numbering scheme for rotors 1-8
% propeller location in the body frame % See Ref. [1] lines 1062-1078
p_b = [ -5.07,  -4.63, -4.63, -5.07,  -19.2,  -18.76,-18.76,-19.2,  -31.94;
       -18.750, -8.45,  8.45, 18.750, -18.750, -8.45,  8.45, 18.750,  0.000;
        -6.73,  -7.04, -7.04, -6.73,   -9.01,  -9.3,  -9.3,  -9.01,  -7.79];
% motor location in the body frame % See Ref. [1] lines 1062-1079
m_b =[  -5.07,  -4.63, -4.63, -5.07,  -19.2,  -18.76,-18.76,-19.2,  -31.94;
       -18.750, -8.45,  8.45, 18.750, -18.750, -8.45,  8.45, 18.750,  0.000;
        -7.60,  -6.04, -6.04, -7.6,   -8.00,  -8.3,  -8.3,  -8.00,  -7.79];

% propeller performance coefficients
prop_coefs = load('APCSF_10x4p7_coef.mat'); % Selected for this configuration

% propeller spin direction
prop_spin = [ +1 -1  +1 -1 -1 +1 -1 +1 +1];

% motor mass
% See Ref. [2] computed as avg (Ex. fpor Rotor mass 1: B17+B18+B37+B38) for all 1-8 rotors
rotor_mass            = 2.66855138; % slugs 
rotor_drive_sys_mass  = 0.70223; % slugs See Ref. [2] cells B128-B135 
rotor_engine_mass     = 1.00984; % slugs See Ref. [2] cells B138-B145
rotor_asbly_mass = rotor_mass + rotor_drive_sys_mass + rotor_engine_mass;

pusher_mass           = 2.129713; % slugs See Ref. [2], sum cells B33+B54+B55
pusher_drive_sys_mass = 2.289138; % slugs See Ref. [2] cell B136
pusher_engine_mass    = 3.31803; % slugs See Ref. [2] cell B146
pusher_asbly_mass = pusher_mass + pusher_drive_sys_mass + pusher_engine_mass;

m_m = [repmat(rotor_asbly_mass,1,8)  pusher_asbly_mass];

% thrust vector % See Ref. [1] for cant_hub angle for each rotor (either 0,
% +8 or -8 degrees)
p_T_e = [ 0,  0.000,  0.000,  0,  0,  0.000,  0.000,  0, 1;
          0, -0.139,  0.139,  0,  0, -0.139,  0.139,  0, 0;
         -1, -0.990, -0.990, -1, -1, -0.990, -0.990, -1, 0];

% build and array of props
Prop = cell(NP,1);
for ii = 1:NP
  prop = PropellerClass( prop_coefs.APCSF_10x4p7_coef,...
                         prop_spin(ii),...
                         D(ii), ...
                         p_b(:,ii), ...
                         m_b(:,ii), ...
                         m_m(ii), ...
                         p_T_e(:,ii));
   Prop{ii} = prop;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rotor Booms Masses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Pusher Prop Engine 
% mass
pusher_drive_sys_mass = 1.7809;
pusher_eng_mass = 13.5389;
pusher_eng_asbly_mass = pusher_drive_sys_mass + pusher_eng_mass;
% inertia matrix
pusher_eng_I = diag([0.000 0.000 0.000]);
% location in the body frame
pusher_eng_cm_b = [ -30.94;
                      0.00;
                     -7.79];
Pusher_Eng = MassClass(pusher_eng_asbly_mass, pusher_eng_I, pusher_eng_cm_b);

% Pusher Prop Generator
% mass
pusher_gen_mass = 4.2643;
% inertia matrix
pusher_gen_I = diag([0.000 0.000 0.000]);
% location in the body frame
pusher_gen_cm_b = [ -30.94;
                      0.00;
                     -7.79];
Pusher_Gen = MassClass(pusher_gen_mass, pusher_gen_I, pusher_gen_cm_b);

% Landing Gear  
% mass
landing_gear_mass = 8.48397; % See Ref. [2] cells B59-B61
% inertia matrix
landing_gear_I = diag([0.000 0.000 0.000]);
% location in the body frame
landing_gear_cm_b = [-13.32464336	-3.0924E-06	-1.391179415]; % See Ref. [2], computed gear CM using cells B59-E61 & B122=E124
Landing_Gear = MassClass(landing_gear_mass, landing_gear_I, landing_gear_cm_b);

% Fuel Tank 1
% mass
fuel_tank_1_mass = 7.9325; % See Ref. [2], sum cells B35+B36
fuel_liq_1_mass = 0.0;
fuel_1_mass = fuel_tank_1_mass + fuel_liq_1_mass;
% inertia matrix
fuel_1_I = diag([0.000 0.000 0.000]);
% location in the body frame
fuel_1_cm_b = [-12.773578 ; 0.00; -3.130658]; % See Ref. [2] cells C35-E36
Fuel_1 = MassClass(fuel_1_mass, fuel_1_I, fuel_1_cm_b);

% Fuel Tank 2
% mass
fuel_tank_2_mass = 1.899172; % See Ref. [2] cell B127
fuel_liq_2_mass = 0.0;
fuel_2_mass = fuel_tank_2_mass + fuel_liq_2_mass;
% inertia matrix
fuel_2_I = diag([0.000 0.000 0.000]);
% location in the body frame
fuel_2_cm_b = [-2.75; 0.00; -4.15]; % See Ref. [2] cell C127-E127
Fuel_2 = MassClass(fuel_2_mass, fuel_2_I, fuel_2_cm_b);

% Systems
sys_mass = 16.223758; % See Ref. [2] cell B126
sys_I = diag([0 0 0]);
sys_cm_b = [ -16; 0; -6]; % See Ref. [2] cells C126-E126
Sys = MassClass(sys_mass, sys_I, sys_cm_b);

% Systems
ext_mass = 54.60; % Remainder of mass (Estimated)
ext_I = diag([0 0 0]);
ext_cm_b = [ -7.6; 0; -0.5]; % Notional location (Estimated)
Ext = MassClass(ext_mass, ext_I, ext_cm_b);

Extra_Mass = {Pusher_Eng Pusher_Gen Landing_Gear Fuel_1 Fuel_2 Sys Ext};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tiltwing Aircraft
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tiltwing = TiltWingClass( WingProp, Tail, Body, Prop, Extra_Mass);

