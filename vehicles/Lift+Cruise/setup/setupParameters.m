function setupParameters(SimIn)
%% Define parameters
% Define parameters below that need to change without recompiling.  You
% will need to replace "SimIn" with "SimPar" in the Simulink diagram.
% 

%% Define Switches parameters
SimPar.Switches = SimIn.Switches;

%% Define Mass Properties parameters
SimPar.MP.mass = SimIn.Model.mass;
SimPar.MP.I    = SimIn.Model.I;

%% Define Control Surface Actuator Parameters
SimPar.Actuator.BW      = SimIn.Act.BW;    % Hz
SimPar.Actuator.Zeta    = SimIn.Act.Zeta; % damping ratio
SimPar.Actuator.RL      = SimIn.Act.RateLim;  % rad/s
SimPar.Actuator.MaxPos  = SimIn.Act.PosLim_hi; % rad
SimPar.Actuator.MinPos  = SimIn.Act.PosLim_lo;  % rad
SimPar.Actuator.Bias    = SimIn.Act.Bias;

%% Define Propulsion Dynamics Parameters
SimPar.Engine.BW      = SimIn.Eng.BW;    % Hz
SimPar.Engine.Zeta    = SimIn.Eng.Zeta; % damping ratio
SimPar.Engine.RL      = SimIn.Eng.RateLim;  % rad/s
SimPar.Engine.MaxPos  = SimIn.Eng.PosLim_hi; % rad
SimPar.Engine.MinPos  = SimIn.Eng.PosLim_lo;  % rad
SimPar.Engine.Bias    = SimIn.Eng.Bias;

%% Define IC Parameters
SimPar.IC = SimIn.IC;
SimPar.startTime = SimIn.startTime;
SimPar.stopTime = SimIn.stopTime;

%% Define EOM IC Parameters
SimPar.EOM = SimIn.EOM;

% Define trajectory file parameter
%   note:  convert to double due to SimPar limitations
% lenTrajFile = length(SimIn.trajFile);
% maxLength = 255;
% if (lenTrajFile <= maxLength)
%   SimPar.trajFile = zeros(1,maxLength); % set max char array size to 255
%   SimPar.trajFile(1:length(SimIn.trajFile)) = double(SimIn.trajFile);
% else
%   error('Error:  Length of path/filename of trajectory file exceeds %d, length = %d', maxLength, lenTrajFile);
% end

%% Define Guidance/Control parameters
%SimPar.Guidance. 
SimPar.Control.Adaptive.AdaptOn = SimIn.Control.Adaptive.AdaptOn;
SimPar.Control.Vel_bIc_0 = SimIn.Control.IC.Vel_bIc_0;

% AGI Allocation parameters
SimPar.Control.Alloc.Num_Mom            = 6; % Nominal number of allocation moments
SimPar.Control.Alloc.Num_Eff            = 14; % Nominal number of allocation effectors
SimPar.Control.Alloc.Alloc_Type_Flag    = 0;  % Delta allocation =0, Absolute Allocation = 1
SimPar.Control.Alloc.Eff_Rate_Flag      = 1; % Enforce effector rate limits=0, don't enforce = 1

%% Define reference input parameters
% need to add trajectory data
if isfield(SimIn.RefInputs,'doublets')
    SimPar.RefInputs.doublets = SimIn.RefInputs.doublets;
    SimPar.RefInputs.ramps = SimIn.RefInputs.ramps;
end
SimPar.RefInputs.initialLat = SimIn.RefInputs.initialLat;
SimPar.RefInputs.initialLon = SimIn.RefInputs.initialLon;
SimPar.RefInputs.initialAlt = SimIn.RefInputs.initialAlt;
SimPar.RefInputs.initialTrack = SimIn.RefInputs.initialTrack;
SimPar.RefInputs.initialVelocity = SimIn.RefInputs.initialVelocity;
if isfield(SimIn.RefInputs,'SimInput')
    SimPar.RefInputs.SimInput = SimIn.RefInputs.SimInput;
    SimPar.RefInputs.SimInput.Vel_bIc_des = [SimPar.RefInputs.SimInput.Vel_bIc_des.Time SimPar.RefInputs.SimInput.Vel_bIc_des.Data];
    SimPar.RefInputs.SimInput.pos_des = [SimPar.RefInputs.SimInput.pos_des.Time SimPar.RefInputs.SimInput.pos_des.Data];
    SimPar.RefInputs.SimInput.chi_des = [SimPar.RefInputs.SimInput.chi_des.Time SimPar.RefInputs.SimInput.chi_des.Data];
    SimPar.RefInputs.SimInput.chi_dot_des = [SimPar.RefInputs.SimInput.chi_dot_des.Time SimPar.RefInputs.SimInput.chi_dot_des.Data];
end
if isfield(SimIn.RefInputs,'Bezier')
    SimPar.RefInputs.Bezier.waypointsX = SimIn.RefInputs.Bezier.waypoints{1};
    SimPar.RefInputs.Bezier.waypointsY = SimIn.RefInputs.Bezier.waypoints{2};
    SimPar.RefInputs.Bezier.waypointsZ = SimIn.RefInputs.Bezier.waypoints{3};
    if iscell(SimIn.RefInputs.Bezier.time_wpts)
        SimPar.RefInputs.Bezier.time_wptsX = SimIn.RefInputs.Bezier.time_wpts{1};
        SimPar.RefInputs.Bezier.time_wptsY = SimIn.RefInputs.Bezier.time_wpts{2};
        SimPar.RefInputs.Bezier.time_wptsZ = SimIn.RefInputs.Bezier.time_wpts{3};
    else 
        SimPar.RefInputs.Bezier.time_wptsX = SimIn.RefInputs.Bezier.time_wpts;
        SimPar.RefInputs.Bezier.time_wptsY = SimIn.RefInputs.Bezier.time_wpts;
        SimPar.RefInputs.Bezier.time_wptsZ = SimIn.RefInputs.Bezier.time_wpts;
    end 
end
%% Define Turbulence 
SimPar.Environment.Turbulence.Vehicle_span_m      = SimIn.Environment.Turbulence.Vehicle_span_m;
SimPar.Environment.Turbulence.RandomSeedLong         = SimIn.Environment.Turbulence.RandomSeeds(1);
SimPar.Environment.Turbulence.RandomSeedLat         = SimIn.Environment.Turbulence.RandomSeeds(2);
SimPar.Environment.Turbulence.RandomSeedVert         = SimIn.Environment.Turbulence.RandomSeeds(3);
SimPar.Environment.Turbulence.RandomSeedPGust         = SimIn.Environment.Turbulence.RandomSeeds(4);
SimPar.Environment.Turbulence.WindAt5kft          = SimIn.Environment.Turbulence.WindAt5kft;
SimPar.Environment.Turbulence.WindDirectionAt5kft = SimIn.Environment.Turbulence.WindDirectionAt5kft;
SimPar.Environment.Turbulence.MeanWindGain        = SimIn.Environment.Turbulence.MeanWindGain;
SimPar.Environment.Turbulence.intesity            = 1; %intensity 1=light, 2=moderate, 3=severe

%% Define Winds Parameters
SimPar.Environment.Winds.Gust_wHh   = SimIn.Environment.Winds.Gust_wHh;% (deg) From North
SimPar.Environment.Winds.Vel_wHh    = SimIn.Environment.Winds.Vel_wHh;% (deg) From North
SimPar.Environment.Winds.VelDtH_wHh = SimIn.Environment.Winds.VelDtH_wHh;% (kts) Total wind speed (assuming zero vertical component).

%% Define Failure Parameters
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
% Failure Types: 1=Hold Last, 2=Pre-Scale, 3=Post-Scale, 4=Pos Limits
% 5=Rate Limits, 6=Generic, 7=Runaway(currently inactive), 8=Control Reversal
SimPar.Fail.Surfaces.FailInit           = [0; 0; 0; 0; 0]; % Specify failure type (see above)
SimPar.Fail.Surfaces.InitTime           = [0; 0; 0; 0; 0]; % Specify failure start time (sim time)
SimPar.Fail.Surfaces.StopTime           = [0; 0; 0; 0; 0]; % Specify failure stop time (use inf for end of sim)
SimPar.Fail.Surfaces.PreScale           = [0; 0; 0; 0; 0]; % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
SimPar.Fail.Surfaces.PostScale          = [0; 0; 0; 0; 0]; % Specify decimal post-scale factor (e.g. 0.10 or 1.3)
SimPar.Fail.Surfaces.PosBias            = zeros(SimIn.Act.nCS,1); % Specify position bias in first-order model
SimPar.Fail.Surfaces.PosScale           = zeros(SimIn.Act.nCS,1); % Specify position scale factor in first-order model
SimPar.Fail.Surfaces.UpPlim             = SimPar.Actuator.MaxPos; % Specify effector upper position limit
SimPar.Fail.Surfaces.LwrPlim            = SimPar.Actuator.MinPos; % Specify effector lower position limit
SimPar.Fail.Surfaces.RateBias           = zeros(SimIn.Act.nCS,1); % Specify rate bias in first-order model
SimPar.Fail.Surfaces.RateScale          = zeros(SimIn.Act.nCS,1); % Specify rate scale factor in first-order model
SimPar.Fail.Surfaces.UpRlim             = SimPar.Actuator.RL; % Specify effector upper rate limit
SimPar.Fail.Surfaces.LwrRlim            = -SimPar.Actuator.RL; % Specify effector lower rate limit
SimPar.Fail.Surfaces.AccelBias          = zeros(SimIn.Act.nCS,1); % Specify acceleration bias in second order model (placeholder not yet implemented)
SimPar.Fail.Surfaces.AccelScale         = zeros(SimIn.Act.nCS,1); % Specify acceleration scale factor in second order model (placeholder not yet implemented)
SimPar.Fail.Surfaces.UpAlim             = zeros(SimIn.Act.nCS,1); % Specify acceleration upper limit (placeholder not yet implemented) 
SimPar.Fail.Surfaces.LwrAlim            = zeros(SimIn.Act.nCS,1); % Specify acceleration lower limit (placeholder not yet implemented) 
SimPar.Fail.Surfaces.Generic_Sig_Select = [zeros(3,1);ones(8,1);zeros(4,1)]; % Specifies which SimPar.Fail parameters to use for generic failure (FailInit == 6), 
 % Columns 1-15 are: Hold_Last, Pre-Scale, Post-Scale, Pos-Bias, Pos-Scale, Upr-Plim, Lwr-Plim,
 % Rate-Bias, Rate-Scale, Upr-RateLim, Lwr-RateLim, Accel-Bias, Accel-Scale, Upr-AccelLim, Lwr-AccelLim 
                                           

 % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% Failure Types: 1=Hold Last, 2=Pre-Scale, 3=Post-Scale, 4=Pos Limits
% 5=Rate Limits, 6=Generic, 7=Runaway(currently inactive), 8=Control Reversal
SimPar.Fail.Props.FailInit           = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify failure type (see above)
SimPar.Fail.Props.InitTime           = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify failure start time (sim time)
SimPar.Fail.Props.StopTime           = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify failure stop time (use inf for end of sim)
SimPar.Fail.Props.PreScale           = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
SimPar.Fail.Props.PostScale          = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify decimal post-scale factor (e.g. 0.10 or 1.3)
SimPar.Fail.Props.PosBias            = zeros(SimIn.Eng.nENG,1); % Specify position bias in first-order model
SimPar.Fail.Props.PosScale           = zeros(SimIn.Eng.nENG,1); % Specify position scale factor in first-order model
SimPar.Fail.Props.UpPlim             = SimPar.Engine.MaxPos; % Specify effector upper position limit
SimPar.Fail.Props.LwrPlim            = SimPar.Engine.MinPos; % Specify effector lower position limit
SimPar.Fail.Props.RateBias           = zeros(SimIn.Eng.nENG,1); % Specify rate bias in first-order model
SimPar.Fail.Props.RateScale          = zeros(SimIn.Eng.nENG,1); % Specify rate scale factor in first-order model
SimPar.Fail.Props.UpRlim             = SimPar.Engine.RL; % Specify effector upper rate limit
SimPar.Fail.Props.LwrRlim            = -SimPar.Engine.RL; % Specify effector lower rate limit
SimPar.Fail.Props.AccelBias          = zeros(SimIn.Eng.nENG,1); % Specify acceleration bias in second order model (placeholder not yet implemented)
SimPar.Fail.Props.AccelScale         = zeros(SimIn.Eng.nENG,1); % Specify acceleration scale factor in second order model (placeholder not yet implemented)
SimPar.Fail.Props.UpAlim             = zeros(SimIn.Eng.nENG,1); % Specify acceleration upper limit (placeholder not yet implemented) 
SimPar.Fail.Props.LwrAlim            = zeros(SimIn.Eng.nENG,1); % Specify acceleration lower limit (placeholder not yet implemented) 
SimPar.Fail.Props.Generic_Sig_Select = [zeros(3,1);ones(8,1);zeros(4,1)]; % Specifies which SimPar.Fail parameters to use for generic failure (FailInit == 6),
 % Columns 1-15 are: Hold_Last, Pre-Scale, Post-Scale, Pos-Bias, Pos-Scale, Upr-Plim, Lwr-Plim,
 % Rate-Bias, Rate-Scale, Upr-RateLim, Lwr-RateLim, Accel-Bias, Accel-Scale, Upr-AccelLim, Lwr-AccelLim 

%%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
%% Bus Objects
props.Description = 'L+C Tunable Parameters';
props.DataScope   = 'Exported';
props.HeaderFile  = 'Parameters.h';
props.Alignment   = -1;% Not used

% props.Elements will be defined in buildBusObject.m.
[b] = buildBusObject(SimPar,'BUS_PARAM_','SimPar',props);
assignin('base',['BUS_PARAM_','SimPar'],b);% % Rename the bus object with prefix

%% Simulink Parameter Object
param = Simulink.Parameter;
param.Value = SimPar;
param.CoderInfo.StorageClass = 'ExportedGlobal';
param.CoderInfo.Alignment = -1;% Not used
param.Description = 'L+C Parameters';
param.DataType =  'Bus: BUS_PARAM_SimPar';
param.Min = [];
param.Max = [];
param.DocUnits = '';

assignin('base','SimPar', param);

end
