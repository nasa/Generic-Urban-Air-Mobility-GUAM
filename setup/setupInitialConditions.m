function IC = setupInitialConditions(SimIn, target)


RefInputs = SimIn.RefInputs;
Control = SimIn.Control;

% Units
s    = SimIn.Units.s;
deg  = SimIn.Units.deg;
ft   = SimIn.Units.ft;

% Set the initial coditions from the reference trajectory
% and the controller settings

Veas      = Control.IC.Vtot0;
Vtas      = Control.IC.Vtot0;
alpha     = Control.IC.alpha0; 
beta      = Control.IC.beta0;
gamma     = Control.IC.gamma0;
track     = Control.IC.chi0;
roll      = Control.IC.eta0(1);
pitch     = Control.IC.eta0(2);
yaw       = Control.IC.chi0; % initally sync yaw angle with track angle

% These will get set if we start in a turn
rollrate  = 0*deg/s;
pitchrate = 0*deg/s;
yawrate   = 0*deg/s;


% Theses are not used anywhere
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pdeg      = 0*deg/s;
qdeg      = 0*deg/s;
rdeg      = 0*deg/s;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% if additional or alternatice IC conditions are passed
% assign them here
if (exist('target','var') && ~isempty(target))
  fields = fieldnames(target);
  numFields = length(fields);
  for i = 1:numFields
    switch fields{i}
      case 'eas'
        Veas = target.eas*ft/s;
      case 'tas'
        Vtas = target.tas*ft/s;
      case 'alpha'
        alpha = target.alpha*deg;
      case 'beta'
        beta = target.beta*deg;
      case 'gamma'
        gamma = target.gamma*deg;
      case 'gndtrack'
        track = target.gndtrack*deg;
      case 'roll'
        roll = target.roll*deg;
      case 'pitch'
        pitch = target.pitch*deg;
      case 'yaw'
        yaw = target.yaw*deg;
      case 'rollrate'
        rollrate = target.rollrate*deg/s;
      case 'pitchrate'
        pitchrate = target.pitchrate*deg/s;
      case 'yawrate'
        yawrate = target.yawrate*deg/s;


      % Theses are not used anywhere
      % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      case 'pdeg'
        pdeg = target.pdeg*deg/s;
      case 'qdeg'
        qdeg = target.qdeg*deg/s;
      case 'rdeg'
        rdeg = target.rdeg*deg/s;
      % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


      otherwise
        error('Unknown target field:  %s\n',fields{i});
    end
  end
  
  if ~isfield(target,'theta')
    pitch = gamma + alpha;
  end
  if ~isfield(target,'yaw')
    yaw = track;
  end
  if ~isfield(target,'eas')
    Veas = Vtas;
  end
end

IC.GrndAltMSL = 0; % (ft) MSL altitude of ground.

% Psuedo-States: Used to initialize EOM
%IC.LatGeod  = 37.0881*deg;  % Latitude (deg)
%IC.Lon      = -76.3448*deg; % Longitude (deg)
%IC.AltMSL   = 0.0*ft;       % (ft) Altitude above Mean Sea Level

IC.LatGeod  = RefInputs.initialLat; % Latitude (rad)
IC.Lon      = RefInputs.initialLon; % Longitude (rad)
IC.AltMSL   = RefInputs.initialAlt;  % (ft) Altitude above Mean Sea Level

IC.Vgrnd    = Vtas;         % (ft/s) Velocity reletive to ground (typically over written by trim) 
IC.Accel    = 0*ft/s^2;     % (ft/s^2) Linear Acceleration
IC.gamma    = gamma;        % (rad) Flight Path Angle 
IC.track    = track;        % (rad) Velocity Azimuth (ground track) 
IC.phi      = roll;         % (rad) Bank angle 
IC.theta    = pitch;        % (rad) Pitch attitude 
IC.psi      = yaw;          % (rad) Heading  %linearization Q2E transformation fails for psi=+-90 deg;
IC.phidot   = rollrate;     % (rad/s) Bank angle rate
IC.thetadot = pitchrate;    % (rad/s) Attitude rate
IC.psidot   = yawrate;      % (rad/s) Heading rate

% The following are placeholderes that will be over written by trim.
IC.ubdot = 0*ft/s^2;  %(ft/s^2) Components of VelDtB_bEb
IC.vbdot = 0*ft/s^2;
IC.wbdot = 0*ft/s^2;

IC.pbdot = 0*deg/s^2;  %(rad/s^2) Body axis angular accels
IC.qbdot = 0*deg/s^2; 
IC.rbdot = 0*deg/s^2; 

IC.Vtot  = IC.Vgrnd;
IC.mu    = 0;
IC.alpha = alpha;
IC.beta  = beta;
IC.Veas  = Veas;
IC.Mach  = 0;
IC.qbar  = 0;
        
% Not sure what these are
IC.alphagrnd = 0;
IC.betagrnd  = 0;

IC.VelDtI_bIi  = [0;0;0];
IC.OmegDtI_BIb = [0;0;0];

IC.VelDtB_bEb  = [0;0;0];
IC.OmegDtB_BHb = [0;0;0];

% set default bias (trim) values for control surfaces
IC.bias.Surfaces    = Control.IC.surf0;

% set default bias (trim) values for engines
IC.bias.Engines = Control.IC.rotor0;



