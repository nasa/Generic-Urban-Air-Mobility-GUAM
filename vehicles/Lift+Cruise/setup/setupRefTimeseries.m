function Out = setupRefTimeseries(SimIn, target)
% L+C reference inputs
arguments
  SimIn struct
  target struct = []
end
 
% setup reference trajectory
traj = target.RefInput;

% set initial reference inputs based on trajectory data
initialVelocity = traj.Vel_bIc_des.Data(1,:);    % ft/s
initialTrack    = traj.chi_des.Data(1,:);

fnames = fieldnames(traj);
refTime = [];
for n=1:length(fnames)
    time = getfield(traj,fnames{n}).Time;
    [s,l]=bounds([refTime,time(1),time(end)] );
    refTime = [s,l]; 
end

%traj.refTime = [s l];
traj.refTime = time;

Out.coord = 'ned';

Out.initialVelocity = initialVelocity;
Out.initialTrack    = initialTrack;
% hardcode initial lat lon until conversion method added
Out.initialLat      = traj.pos_des.Data(1,1);%37.09;
Out.initialLon      = traj.pos_des.Data(1,2);%-76.39;
Out.initialAlt      = traj.pos_des.Data(1,3);

Out.trajectory      = traj;
Out.SimInput.Vel_bIc_des        = traj.Vel_bIc_des;
Out.SimInput.pos_des            = traj.pos_des;
Out.SimInput.chi_des            = traj.chi_des;
Out.SimInput.chi_dot_des        = traj.chi_dot_des;
end
