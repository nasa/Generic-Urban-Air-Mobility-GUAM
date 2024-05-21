
bUserExist = exist('userStruct','var');
if bUserExist
  bExist = isfield(userStruct, 'switches');
else
  bExist = false;
end

if bExist
  disp('userStruct.switches exists');
else
  disp('userStruct.switches does not exist');
end

% set default switch settings
SimIn.Switches.WindShearOn       = 0;
SimIn.Switches.SensorNoiseOn     = 0;
SimIn.Switches.TrimModeOn        = 0;
SimIn.Switches.LinearizeModeOn   = 0;
SimIn.Switches.RefTrajOn         = 0; % Switch on forces switches FeedbackCurrent and PositionError
SimIn.Switches.FeedbackCurrent   = 1; % Turns on/off table lookup (controller gains etc.) for current velocities
SimIn.Switches.PositionError     = 0; % Turns on/off position/heading error feedback in generalized controller
SimIn.Switches.TurbulenceOn      = 0; % std Ubody, Vbody, Wbody turbulence
SimIn.Switches.RotationalGustsOn = 0; % Pbody, Qbody, Rbody rotational turbulence
SimIn.Switches.WindsOn           = 0; % Steady Winds, Wind shear, etc..
SimIn.Switches.AeroPropDeriv     = 0;
SimIn.Switches.TrajAbeamPos      = 0;

fprintf('\n---------------------------------------\n')
fprintf('Switch setup:\n');

% select WindShearOn
if (bExist && isfield(userStruct.switches,'WindShearOn'))
  SimIn.Switches.WindShearOn = userStruct.switches.WindShearOn;
end
% select SensorNoiseOn
if (bExist && isfield(userStruct.switches,'SensorNoiseOn'))
  SimIn.Switches.SensorNoiseOn = userStruct.switches.SensorNoiseOn;
end
% select refTrajOn
% if (bExist && isfield(userStruct.switches,'RefTrajOn'))
%   SimIn.Switches.RefTrajOn = userStruct.switches.RefTrajOn;
% else
%   %SimIn.Switches.RefTrajOn = input('Reference Trajectory <0 = OFF, 1 = ON>: ');
%   SimIn.Switches.RefTrajOn = 0;
% end
if (SimIn.vehicleType == VehicleEnum.LiftPlusCruise) && ((SimIn.refInputType == RefInputEnum.TIMESERIES) || (SimIn.refInputType == RefInputEnum.BEZIER))
  SimIn.Switches.RefTrajOn = 1;
end
% select FeedbackCurrent
if (bExist && isfield(userStruct.switches,'FeedbackCurrent'))
  SimIn.Switches.FeedbackCurrent = userStruct.switches.FeedbackCurrent;
end
% select PositionError
if (bExist && isfield(userStruct.switches,'PositionError'))
  SimIn.Switches.PositionError = userStruct.switches.PositionError;
end
% select TurbulenceOn
if (bExist && isfield(userStruct.switches,'TurbulenceOn'))
  SimIn.Switches.TurbulenceOn = userStruct.switches.TurbulenceOn;
end
% select RotationalGustsOn
if (bExist && isfield(userStruct.switches,'RotationalGustsOn'))
  SimIn.Switches.RotationalGustsOn = userStruct.switches.RotationalGustsOn;
end
% select WindsOn
if (bExist && isfield(userStruct.switches,'WindsOn'))
  SimIn.Switches.WindsOn = userStruct.switches.WindsOn;
end
% select AeroPropDeriv
if (bExist && isfield(userStruct.switches,'AeroPropDeriv'))
  SimIn.Switches.AeroPropDeriv = userStruct.switches.AeroPropDeriv;
end
% select TrajAbeamPos
if (bExist && isfield(userStruct.switches,'TrajAbeamPos'))
  SimIn.Switches.TrajAbeamPos = userStruct.switches.TrajAbeamPos;
end

%{
fprintf('\n---------------------------------------');
fprintf('\n**  WindShearOn Switch:       %d **', SimIn.Switches.WindShearOn);
fprintf('\n**  SensorNoiseOn Switch:     %d **', SimIn.Switches.SensorNoiseOn);
fprintf('\n**  RefTrajOn Switch:         %d **', SimIn.Switches.RefTrajOn);
fprintf('\n**  FeedbackCurrent Switch:   %d **', SimIn.Switches.FeedbackCurrent);
fprintf('\n**  PositionError Switch:     %d **', SimIn.Switches.PositionError);
fprintf('\n**  TurbulenceOn Switch:      %d **', SimIn.Switches.TurbulenceOn);
fprintf('\n**  RotationalGustsOn Switch: %d **', SimIn.Switches.RotationalGustsOn);
fprintf('\n**  WindsOn Switch:           %d **', SimIn.Switches.WindsOn);
fprintf('\n**  AeroPropDeriv Switch:     %d **', SimIn.Switches.AeroPropDeriv);
fprintf('\n**  TrajAbeamPos Switch:      %d **', SimIn.Switches.TrajAbeamPos);
fprintf('\n---------------------------------------\n');
%}

clear bUserExist bExist
