function Out = setupVehicle(SimIn, target)
arguments
  SimIn struct
  target struct = []
end

s      = SimIn.Units.s;

% Run setupRef based on RefInputEnum type
if SimIn.refInputType == RefInputEnum.TIMESERIES 
    SimIn.RefInputs = setupRefTimeseries(SimIn, target);
elseif SimIn.refInputType == RefInputEnum.BEZIER
    SimIn.RefInputs = setupRefBezier(SimIn, target);
else
    SimIn.RefInputs = setupRefInputs(SimIn, target);
end

% default stop time from reference trajectory
if SimIn.refInputType == RefInputEnum.TIMESERIES 
    stopTime = SimIn.RefInputs.trajectory.refTime(end);
elseif SimIn.refInputType == RefInputEnum.BEZIER 
    stopTime = SimIn.RefInputs.Bezier.time_wpts{1}(end);
end

% check for any target overrides
if (exist('target','var') && ~isempty(target))
  fields = fieldnames(target);
  numFields = length(fields);
  for i = 1:numFields
    switch fields{i}
      % not worrying about lat/lon currently
      case 'alt'
      case 'tas'
      case 'gndtrack'
      case 'RefInput'
      case 'Rate'
        % Now set any rates prescribed in the target
        fnames = fieldnames(target.Rate);
        for i = 1:length(fnames)
            SimIn.Rate = setfield(SimIn.Rate,fnames{i},target.Rate.(fnames{i}));
        end
        case 'stopTime'
            stopTime = target.stopTime* s; % s;
      otherwise
        error('Unknown target field:  %s\n',fields{i});
    end
  end
end
if SimIn.refInputType == RefInputEnum.TIMESERIES
    SimIn.startTime = SimIn.RefInputs.trajectory.refTime(1);  % zero-based time value, not trajectory time
elseif SimIn.refInputType == RefInputEnum.BEZIER
    SimIn.startTime = SimIn.RefInputs.Bezier.time_wpts{1}(1);
else
    SimIn.startTime = 0;
end
SimIn.stopTime = stopTime;

% setup the baseline_1 Controller gains
SimIn.Control = setupControl(SimIn);

% Set initial conditions from the reference trajectory
SimIn.IC          = setupInitialConditions(SimIn,[]);
SimIn.EOM         = setupEOM(SimIn);  % Must come after Env and IC's
SimIn.Trim        = setupTrim(SimIn);
SimIn.Act         = setupActuators(SimIn);
SimIn.Eng         = setupEngines(SimIn);

if SimIn.sensorType ~= SensorsEnum.None
  SimIn.Sensor    = setupSensors(SimIn);
end

Out = SimIn;

end
