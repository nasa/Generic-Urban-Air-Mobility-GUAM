function Out = setupEnvironment(SimIn)
%% Define Environment
% Specify Earth parameters

% *************************************************************************
% Modifications:
% 3-22-2023 MJA: Updated script with publicly releasable source material
% Earth related quantities are provided by unclassified, publicly
% releaseable from the National Geospace Intelligence Agency standard:
% NGA.STND.0036_1.0.0_WGS84 which can be found at:
% https://nsgreg.nga.mil/doc/view?i=4085&month=3&day=21&year=2023
% *************************************************************************


  % Establish baseline units
  units = SimIn.Units;
  m = units.m;
  ft = units.ft;
  s = units.s;
  ft3_s2 = ft^3/s^2;

  Out.Earth.ERA_IC = 0;%Initial Earth Rotation Angle (rad)
  Out.Earth.GeoidHeight = 0;

  % WGS 84 Four Defining Parameters,  
  % NGA.STND.0036_1.0.0_WGS84
  % Section: 3.4.4, Table 3.1
  a = 6378137.0 * m;             % (m)       Semi-major Axis
  f = 1 / 298.257223563;         % (n/a)     Flattening parameter
  Omega = 7292115.0e-11 / s;     % (rad/s)   Angular Velocity of the Earth
  GM = 3986004.418e8 * (m^3/s^2);% (m^3/s^2) Earth's Gravitational Constant (Mass of Earth's Atmosphere Included)

  % NGA.STND.0036_1.0.0_WGS84
  % Section: 5.2, Table 5.1
  Cn0 = [
  -0.484165143790815E-03;% Normalized Second degree Zonal Harmonic
   0.957161207093473E-06;
   0.539965866638991E-06;
   0.686702913736681E-07;
  -0.149953927978527E-06;
   0.905120844521618E-07;
   0.494756003005199E-07
  ];

  % Derived Parameters
  b = a*(1-f);      % semi-minor axis
  e = sqrt(2*f-f*f);% eccentricity
  J = -Cn0.*sqrt(2*[2:8]'+1); 

  Out.Earth.RadiusEquator = a;
  Out.Earth.RadiusPoles   = b;
  Out.Earth.Eccentricity  = e;
  Out.Earth.Flattening    = f;
  Out.Earth.Omega         = Omega;

  % Gravity
  Out.Earth.Gravity.mu     = GM;
  Out.Earth.Gravity.Jcoefs = J;    % vector of coefficients
  Out.Earth.Gravity.g0 = [0;0;units.g0]*ft/s^2;

  Out.Earth.AltPresGrnd  = 0*ft;
  Out.Earth.AltGeodGrnd  = 0*ft;

  Out.Atmos      = setupAtmosphere(SimIn);
  Out.Winds      = setupWinds(SimIn);
  Out.Turbulence = setupTurbulence(SimIn);
  Out.Runway     = setupRunway(SimIn);

end
