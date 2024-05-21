%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType = 1; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)

%% Define the target structure and provide the Ramp settings
target = struct('tas', 0, 'gndtrack', 0,'stopTime', 30);

% Initialize the sim
simSetup;

% Specify the Ramp settings
SimPar_Set_Ramps; % Sets a basic ramp program
% SimPar_Null_Ramps; % Nulls out the ramps (aircraft stays trimmed)

% Open the simulation
open(model);