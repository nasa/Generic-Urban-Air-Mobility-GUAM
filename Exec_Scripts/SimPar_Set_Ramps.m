% Set Desired Ramps (RefInput profiles)
s   = SimIn.Units.s;
d2r = SimIn.Units.deg;

% SimPar.Value.RefInputs.ramps.Vel_startTime1     = [5; 0; 10] * s; 
% SimPar.Value.RefInputs.ramps.Vel_slope1         = [2; 0; -1;]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime2     = [85; 0; 17] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope2         = [-2; 0; 1;]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime3     = [90; 0; 90] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope3         = [-2; 0; 1]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime4     = [170; 0; 97] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope4         = [2; 0; -1]; % ft/s
% SimIn.stopTime = 175; % Set the sim stop time (in seconds)


% *************************************************************************
% % Example hover=> climb and accelerate => level off => decel and descend
% % profile 
% SimPar.Value.RefInputs.ramps.Vel_startTime1     = [5; 0; 10] * s; 
% SimPar.Value.RefInputs.ramps.Vel_slope1         = [2; 0; 1;]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime2     = [60; 0; 16] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope2         = [-1; 0; -1;]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime3     = [155; 0; 96] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope3         = [-2.5; 0; -1]; % ft/s
% SimPar.Value.RefInputs.ramps.Vel_startTime4     = [235; 0; 102] * s; % s
% SimPar.Value.RefInputs.ramps.Vel_slope4         = [1.5; 0; 1]; % ft/s
SimIn.stopTime = 250; % Set the sim stop time (in seconds)

% *************************************************************************
% Example hover=>  accelerate  => decel 
% profile
SimPar.Value.RefInputs.ramps.Vel_startTime1     = [5; 0; 5] * s; 
SimPar.Value.RefInputs.ramps.Vel_slope1         = [1; 0; 1]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime2     = [205; 0; 11] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope2         = [-1; 0; -1]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime3     = [210; 0; 210] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope3         = [-1; 0; -1]; % ft/s
SimPar.Value.RefInputs.ramps.Vel_startTime4     = [410; 0; 222] * s; % s
SimPar.Value.RefInputs.ramps.Vel_slope4         = [1; 0; 1]; % ft/s
SimIn.stopTime = 420; % Set the sim stop time (in seconds)

% Set the steady winds
SimPar.Value.Environment.Turbulence.WindAt5kft          = 0; % Knots
SimPar.Value.Environment.Turbulence.WindDirectionAt5kft = 0; % True