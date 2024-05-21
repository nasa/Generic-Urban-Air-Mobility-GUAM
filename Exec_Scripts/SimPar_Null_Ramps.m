s   = SimIn.Units.s;
d2r = SimIn.Units.deg;

SimPar.Value.RefInputs.ramps.Vel_startTime1         = [0; 0; 0] * s; 
SimPar.Value.RefInputs.ramps.Vel_slope1             = [0; 0; 0;]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime2         = [0; 0; 0] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope2             = [0; 0; 0;]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime3         = [0; 0; 0] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope3             = [0; 0; 0;]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime4         = [0; 0; 0] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope4             = [0; 0; 0;]; % ft/s

SimPar.Value.RefInputs.doublets.Vel_stepTime        = [0; 0; 0] * s; % s
SimPar.Value.RefInputs.doublets.Vel_initialValue    = [0; 0; 0];
SimPar.Value.RefInputs.doublets.Vel_finalValue      = [0; 0; 0];
SimPar.Value.RefInputs.doublets.Track_stepTime      = 0 * s; % s
SimPar.Value.RefInputs.doublets.Track_initialValue  = 0;
SimPar.Value.RefInputs.doublets.Track_finalValue    = 0;

% Set the steady winds
SimPar.Value.Environment.Turbulence.WindAt5kft          = 0; % Knots
SimPar.Value.Environment.Turbulence.WindDirectionAt5kft = 0; % True