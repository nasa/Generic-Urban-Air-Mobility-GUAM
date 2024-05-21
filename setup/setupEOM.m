function Out = setupEOM(SimIn)
%
% $Id: setupEOM.m 1422 2016-06-13 14:52:49Z jkcarbon $
% Environment:  Flat Earth conditions set.

deg = SimIn.Units.deg;
ft  = SimIn.Units.ft;
s   = SimIn.Units.s;

env = SimIn.Environment;
IC  = SimIn.IC;

%%  Initial EOM states (position, attitude, etc)

% Initialize from given earth-relative states
% Note: LLA is referenced to the CG
 Out.IC_Lat = 0*IC.LatGeod;% initial latitude (rad)
 Out.IC_Lon = 0*IC.Lon;% initial longitude (rad)
% Out.IC_Lat = IC.LatGeod;% initial latitude (rad)
% Out.IC_Lon = IC.Lon;% initial longitude (rad)
Out.IC_alt = IC.AltMSL;% + 0*env.Earth.GeoidHeight;% initial altitude (ft) of center of mass above ellipsoid

V     = IC.Vgrnd;
gamma = IC.gamma;
chi   = IC.track;
phi   = IC.phi; 
theta = IC.theta;
psi   = IC.psi;
alpha = IC.alpha;
beta  = IC.beta;

phidot   = IC.phidot; 
thetadot = IC.thetadot; 
psidot   = IC.psidot; 

pbdot  = IC.pbdot; 
qbdot  = IC.qbdot; 
rbdot  = IC.rbdot; 

ubdot = IC.ubdot;
vbdot = IC.vbdot;
wbdot = IC.wbdot;

Omeg_EIi = 0*[0; 0; env.Earth.Omega];% earth rotation in ECI / ECEF (zero for flat earth)
%Omeg_EIe = Omeg_EIi;

% Attitude
qe2i = QrotZ(-0*env.Earth.ERA_IC)';% inital ECEF to ECI transform (4x1)
qe2h = Qmult(QrotZ(Out.IC_Lon),QrotY(-0*pi/2 - Out.IC_Lat))';% Transform ECEF to NED
qh2b = QmultSeq(QrotZ(psi),QrotY(theta),QrotX(phi))';% Transform from NED to Body
qi2h = Qmult(Qinvert(qe2i),qe2h);                    % Transform from Inertial to NED
qi2b = QmultSeq(Qinvert(qe2i),qe2h,qh2b);            % Transform from ECI to body

qi2n = QmultSeq(qi2h,QrotZ(chi),QrotY(gamma));qi2n = qi2n(:);
qn2b = Qmult(Qinvert(qi2n),qi2b);qn2b = qn2b(:);

% Position
Pos_bee = [0;0;-Out.IC_alt];%llh2ecef(Out.IC_Lat, Out.IC_Lon, Out.IC_alt, env.Earth);% ECEF position
Pos_bei = Qtrans(qe2i,Pos_bee);

if SimIn.refInputType == 'TIMESERIES' || SimIn.refInputType == 3
    Pos_bii = SimIn.RefInputs.trajectory.pos_des.Data(1,:)';
elseif SimIn.refInputType == 'BEZIER' || SimIn.refInputType == 4
    Pos_bii = cellfun(@(x) x(1,1),SimIn.RefInputs.Bezier.waypoints)';
else
    Pos_bii = Pos_bei;
end

% Velocity
Vel_bEh = setupNedVelocity(SimIn, qh2b);
Vel_bEe = Qtrans(Qinvert(qe2h),Vel_bEh);% Initial ECEF velocity
Vel_bEi = Qtrans(qe2i,Vel_bEe);
Vel_bEb = Qtrans(qi2b,Vel_bEi);
Vel_bIi = Qtrans(qe2i,Vel_bEe) + cross(Omeg_EIi,Pos_bei) ;% ECI vel

% Angular velocity
a = env.Earth.RadiusEquator;
e = env.Earth.Eccentricity;
slatgeod = sin(IC.LatGeod);
M = a*(1- e^2)/(1-(e*slatgeod)^2)^(3/2);
N = a/(1-(e*slatgeod)^2)^(1/2);
h = Out.IC_alt;

% Match attitude with rotating oblate planet
Omeg_EIb = Qtrans(qi2b,Omeg_EIi);% Angular rate of the planet relative to inertial frame
Omeg_HEb = 0*Qtrans(qh2b,[Vel_bEh(2)/(N+h);-Vel_bEh(1)/(M+h);-Vel_bEh(2)*tan(IC.LatGeod)/(N+h)]);% Angular rate due to curvature of planet relative to planet frame(zero for flat earth)
Omeg_BHb = [1  0                -sin(theta);
            0  cos(phi) sin(phi)*cos(theta);
            0 -sin(phi) cos(phi)*cos(theta)]*[phidot;thetadot;psidot];% Angular rate of body frame relaltive to local NED frame.
Omeg_BIb = Omeg_BHb + Omeg_HEb + Omeg_EIb;  % Angular rate of body frame relative to inertial frame (rad/s)
Omeg_BEb = Omeg_BHb + Omeg_HEb;% Angular rate of body frame relative to planet frame

% Linear Acceleration
VelDtB_bEb = [ubdot;vbdot;wbdot];
VelDtB_bEi = Qtrans(Qinvert(qi2b),VelDtB_bEb);
VelDtI_bIi = VelDtB_bEi...
          + Qtrans(Qinvert(qi2b),cross(Omeg_BEb,Vel_bEb))...
          + 2*cross(Omeg_EIi,Vel_bIi)...
          - cross(Omeg_EIi,cross(Omeg_EIi,Pos_bei));
Accel_bIi = VelDtI_bIi;
% grav = env.Earth.Gravity;% gravity model parameters
% g_i = J8Grav(PECI,env.Earth.RadiusEquator,grav.mu,grav.Jcoefs);
g_i = Qtrans(Qmult(Qinvert(qe2h),qe2i),env.Earth.Gravity.g0);% NED gravity vector to ECI
Asensed_bIb = Qtrans(Qinvert(qi2b),Accel_bIi - g_i);% initial sensed acceleration 
 
% Angular acceleration 
OmegDtB_BHb = [pbdot;qbdot;rbdot];%
% QDt_i2e  = 0.5*Qmult(Q_i2e,[0;Omeg_EIe]);
% [OmgDtH_HEh,~] = Omeg_HEh_PD(VelDtI_bIi,Vel_bIi,QDt_i2e,Vel_bIi,Pos_bii,Q_i2e,SimIn);
OmegDtH_HEh = [0;0;0]; %(zero for flat earth)
OmegDtH_HEb = Qtrans(qh2b,OmegDtH_HEh);
OmegDtB_HEb = OmegDtH_HEb - cross(Omeg_BEb, Omeg_HEb);
OmegDtB_EIb = -cross(Omeg_BIb, Omeg_EIb);
OmegDtI_BIb = OmegDtB_BHb+(OmegDtB_HEb + OmegDtB_EIb);

%% Outputs
Out.Q_e2h = qe2h;% Transform ECEF to NED
Out.Q_i2h = qi2h;% Transform from Inertial to NED
Out.Q_i2b = qi2b;% Transform from ECI to body
Out.Q_i2n = qi2n;% Transform from ECI to velocity reference frame
Out.Q_n2b = qn2b;% Transform from velocity reference frame to body frame

Out.Pos_bee = Pos_bee;% ECEF position
Out.Pos_bii = Pos_bii;% ECI position

Out.Vel_bEi = Vel_bEi;
Out.Vel_bIi = Vel_bIi;
Out.Vel_bEb = Vel_bEb;

Out.Omeg_BIb = Omeg_BIb;
Out.Omeg_BHb = Omeg_BHb;
Out.Omeg_HEb = Omeg_HEb;
Out.Omeg_EIb = Omeg_EIb;
Out.Omeg_BEb = Omeg_BEb;

Out.OmegDtB_HEb = OmegDtB_HEb;
Out.OmegDtI_BIb = OmegDtI_BIb;

Out.Accel_bIi = Accel_bIi;
Out.Asensed_bIb = Asensed_bIb;% initial sensed acceleration
