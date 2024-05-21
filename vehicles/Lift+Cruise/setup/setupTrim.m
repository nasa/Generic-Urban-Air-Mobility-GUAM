function Trim = setupTrim(SimIn)
% Set defaults for LC trim structure
%
% $Id: setupTrim.m 2584 2017-08-02 21:11:20Z bbacon $

Trim.Set.Vel_bEh = 'VGC';
rpm = 2*pi/60;

%% Set Default Psuedo States
Trim.States.Vgrnd.perturb = false;
Trim.States.Vgrnd.x0      = SimIn.IC.Vgrnd;
Trim.States.Vgrnd.min     =  0*SimIn.Units.ft/SimIn.Units.s;
Trim.States.Vgrnd.max     =  400*SimIn.Units.ft/SimIn.Units.s;
    
Trim.States.gamma.perturb = false;
Trim.States.gamma.x0      = SimIn.IC.gamma;
Trim.States.gamma.min     = -90*SimIn.Units.deg;
Trim.States.gamma.max     =  90*SimIn.Units.deg;
    
Trim.States.track.perturb = false;
Trim.States.track.x0      = SimIn.IC.track;
Trim.States.track.min     = -180*SimIn.Units.deg;
Trim.States.track.max     =  180*SimIn.Units.deg;
    
Trim.States.phidot.perturb = false;
Trim.States.phidot.x0      = SimIn.IC.phidot;
Trim.States.phidot.min     = -90*SimIn.Units.deg/SimIn.Units.s;
Trim.States.phidot.max     =  90*SimIn.Units.deg/SimIn.Units.s;

Trim.States.thetadot.perturb = false;
Trim.States.thetadot.x0      = SimIn.IC.thetadot;
Trim.States.thetadot.min     = -90*SimIn.Units.deg/SimIn.Units.s;
Trim.States.thetadot.max     =  90*SimIn.Units.deg/SimIn.Units.s;

Trim.States.psidot.perturb = false;
Trim.States.psidot.x0      = SimIn.IC.psidot;
Trim.States.psidot.min     = -90*SimIn.Units.deg/SimIn.Units.s;
Trim.States.psidot.max     =  90*SimIn.Units.deg/SimIn.Units.s;

Trim.States.phi.perturb = false;
Trim.States.phi.x0      = SimIn.IC.phi;
Trim.States.phi.min     = -180*SimIn.Units.deg;
Trim.States.phi.max     =  180*SimIn.Units.deg;

Trim.States.theta.perturb = false;
Trim.States.theta.x0      = SimIn.IC.theta;
Trim.States.theta.min     = -20*SimIn.Units.deg;
Trim.States.theta.max     =  20*SimIn.Units.deg;

Trim.States.psi.perturb = false;
Trim.States.psi.x0      = SimIn.IC.psi;
Trim.States.psi.min     = -180*SimIn.Units.deg;
Trim.States.psi.max     =  180*SimIn.Units.deg;

Trim.States.alpha.perturb = false;
Trim.States.alpha.x0      = SimIn.IC.alpha;
Trim.States.alpha.min     = -180*SimIn.Units.deg;
Trim.States.alpha.max     =  180*SimIn.Units.deg;

Trim.States.beta.perturb = false;
Trim.States.beta.x0      = SimIn.IC.beta;
Trim.States.beta.min     = -90*SimIn.Units.deg;
Trim.States.beta.max     =  90*SimIn.Units.deg;

Trim.States.Vtot.perturb = false;
Trim.States.Vtot.x0      = SimIn.IC.Vtot;
Trim.States.Vtot.min     =   0*SimIn.Units.ft/SimIn.Units.s;
Trim.States.Vtot.max     = 400*SimIn.Units.ft/SimIn.Units.s;

%% Set Default Pseudo States
% State derivation defined in trimEOM_LC.m.
Trim.Derivatives.VelDtB_bEb.constrain = [false; false; false];
Trim.Derivatives.VelDtB_bEb.dx0       = [0;0;0]*SimIn.Units.ft/SimIn.Units.s^2;

Trim.Derivatives.OmgDtB_BHb.constrain = [false; false; false];
Trim.Derivatives.OmgDtB_BHb.dx0       = [0;0;0]*SimIn.Units.deg/SimIn.Units.s^2;

Trim.Derivatives.EDt.constrain        = [false; false; false];
Trim.Derivatives.EDt.dx0              = [0;0;0]*SimIn.Units.deg/SimIn.Units.s;

Trim.Derivatives.Veasdot.constrain    = false;
Trim.Derivatives.Veasdot.dx0          = 0*SimIn.Units.ft/SimIn.Units.s^2;

Trim.Derivatives.gammadot.constrain   = false;
Trim.Derivatives.gammadot.dx0         = 0*SimIn.Units.deg/SimIn.Units.s;

Trim.Derivatives.chidot.constrain     = false;
Trim.Derivatives.chidot.dx0           = 0*SimIn.Units.deg/SimIn.Units.s;

Trim.Derivatives.Vtotdot.constrain    = false;
Trim.Derivatives.Vtotdot.dx0          = 0*SimIn.Units.ft/SimIn.Units.s^2;

Trim.Derivatives.alphadot.constrain   = false;
Trim.Derivatives.alphadot.dx0         = 0*SimIn.Units.deg/SimIn.Units.s;

Trim.Derivatives.betadot.constrain    = false;
Trim.Derivatives.betadot.dx0          = 0*SimIn.Units.deg/SimIn.Units.s;

Trim.Derivatives.hddot.constrain      = false;
Trim.Derivatives.hddot.dx0            = 0*SimIn.Units.ft/SimIn.Units.s^2;

%% Outputs
% Signals passed through the root level output port.
Trim.Outputs.Veas.target     = false;
Trim.Outputs.Veas.y0         = SimIn.IC.Veas;

Trim.Outputs.Vtot.target     = false;
Trim.Outputs.Vtot.y0         = SimIn.IC.Vtot;

Trim.Outputs.alpha.target    = false;
Trim.Outputs.alpha.y0        = SimIn.IC.alpha; 

Trim.Outputs.beta.target     = false;
Trim.Outputs.beta.y0         = SimIn.IC.beta; 

Trim.Outputs.gamma.target    = false;
Trim.Outputs.gamma.y0        = SimIn.IC.gamma;

Trim.Outputs.gndtrack.target = false;
Trim.Outputs.gndtrack.y0     = SimIn.IC.track;

Trim.Outputs.Asensed_bIb.target = [false;false;false];
Trim.Outputs.Asensed_bIb.y0     = SimIn.EOM.Asensed_bIb;

Trim.Outputs.Vtotdot.target  = false;
Trim.Outputs.Vtotdot.y0      = 0;

Trim.Outputs.alphadot.target = false;
Trim.Outputs.alphadot.y0     = 0;

Trim.Outputs.betadot.target  = false;
Trim.Outputs.betadot.y0      = 0;

Trim.Outputs.gammadot.target = false;
Trim.Outputs.gammadot.y0     = 0;

Trim.Outputs.chidot.target   = false;
Trim.Outputs.chidot.y0       = SimIn.IC.psidot;

Trim.Outputs.VelDtH_bEh.target = [false;false;false];
Trim.Outputs.VelDtH_bEh.y0     = [0;0;0];

Trim.Outputs.phi.target      = false;
Trim.Outputs.phi.y0          = SimIn.IC.phi;

Trim.Outputs.theta.target    = false;
Trim.Outputs.theta.y0        = SimIn.IC.theta;

Trim.Outputs.psi.target      = false;
Trim.Outputs.psi.y0          = SimIn.IC.psi;

%% Inputs
% Signals passed in through the root level input port.
% will modify to throttle setting once engine dynamics are added, for now, using thrust
Trim.numEngines = SimIn.numEngines;
Trim.Inputs.engines.perturb = boolean(zeros(Trim.numEngines));
Trim.Inputs.engines.u0      = zeros(Trim.numEngines,1);
Trim.Inputs.engines.min     = zeros(Trim.numEngines,1);
Trim.Inputs.engines.max     = repmat(1000,Trim.numEngines,1); % may want set numbers outside the setup script and just set sizes here

% number of trim surfaces is different because some are ganged together
Trim.numSurfaces = 4; % [flap ail elev rudder] 
Trim.surfaceAlloc = [SimIn.Control.surface_alloc.alloc_flap,...
                     SimIn.Control.surface_alloc.alloc_ail,...
                     SimIn.Control.surface_alloc.alloc_elev,...
                     SimIn.Control.surface_alloc.alloc_rud];

Trim.Inputs.surfaces.perturb = boolean(zeros(Trim.numSurfaces));% may want set numbers outside the setup script and just set sizes here
Trim.Inputs.surfaces.u0      =  zeros(Trim.numSurfaces,1)*SimIn.Units.deg;
Trim.Inputs.surfaces.min     =  repmat(-30,Trim.numSurfaces,1)*SimIn.Units.deg;
Trim.Inputs.surfaces.max     =  repmat(30,Trim.numSurfaces,1)*SimIn.Units.deg;



