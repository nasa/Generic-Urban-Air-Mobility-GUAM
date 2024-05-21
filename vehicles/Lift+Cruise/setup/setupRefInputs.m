function Out = setupRefInputs(SimIn, target)
% L+C reference inputs
arguments
  SimIn struct
  target struct = []
end
 
% Units
s      = SimIn.Units.s;
deg    = SimIn.Units.deg;
ft     = SimIn.Units.ft;
kt2fps = SimIn.Units.knot;

% setup reference trajectory
%traj = setupRefTrajectory(SimIn);

%Out.trajectory = traj;
traj.initialLat     = 36.9 * deg;     % rad
traj.initialLon     = 76.2 * deg;     % rad
traj.initialAlt     = 20 * ft;      % ft
traj.initialTrack   = 050 * deg; % rad
traj.Vel_bIc        = [0;0;0];    % ft/s
traj.Time           = [0 100]; % s

Out.trajectory = traj; % Assign to trajectory

% setup doublet inputs
Out.doublets = setupDoublets(SimIn);
Out.ramps    = setupRamps(SimIn);

% set initial reference inputs based on trajectory data
initialLat = traj.initialLat * deg;     % rad
initialLon = traj.initialLon * deg;     % rad
initialAlt = traj.initialAlt * ft;      % ft
initialTrack = traj.initialTrack * deg; % rad
initialVelocity = traj.Vel_bIc(1,:);    % ft/s

% check for any target overrides
if (exist('target','var') && ~isempty(target))
  fields = fieldnames(target);
  numFields = length(fields);
  for i = 1:numFields
    switch fields{i}
      % not worrying about lat/lon currently
      case 'alt'
        initialAlt = target.alt * ft; % ft

      case 'tas'
        target.tas
        Vtas = target.tas * kt2fps;   % ft/s
        % create uvw vector
        initialVelocity = [Vtas; 0; 0];

      case 'gndtrack'
        initialTrack = target.gndtrack * deg; % rad

      case 'stopTime'
        traj.Time = [0 target.stopTime] * s; % s seconds
      otherwise
        error('Unknown target field:  %s\n',fields{i});
    end
  end
end

Out.initialLat = initialLat;
Out.initialLon = initialLon;
Out.initialAlt = initialAlt;
Out.initialTrack = initialTrack;
Out.initialVelocity = initialVelocity;

end
