function Out = setupRefBezier(SimIn, target)
% L+C reference inputs
arguments
  SimIn struct
  target struct = []
end
 
% setup reference trajectory
traj = target.RefInput;

Out.initialVelocity = traj.Vel_bIc_des;
Out.initialTrack    = traj.chi_des;
% Hardcode initial lat lon until conversion method added
Out.initialLat      = traj.pos_des(1);
Out.initialLon      = traj.pos_des(2);
Out.initialAlt      = traj.pos_des(3);

Out.trajectory      = traj.trajectory;

Out.Bezier          = target.RefInput.Bezier;

end
