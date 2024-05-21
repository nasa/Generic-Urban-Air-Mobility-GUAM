function Out = setupRefTrajectory(SimIn)
% L+C reference trajectory, based on EUTL specification

import gov.nasa.larcfm.*

r2d = 1/SimIn.Units.deg;
m2ft = SimIn.Units.m;
kt2fps = SimIn.Units.knot;

% create a Waypoint from lat/lon string
%wpt = ATM.WayPoint.make('N402012W0891260')

% default L+C reference trajectory
%trajFile = './vehicles/Lift+Cruise/ReferenceTrajectories/Plan_UF36_CL86_15CA_20200306_0836.txt'
%trajFile = './vehicles/Lift+Cruise/ReferenceTrajectories/Plan_UF36_CL86_15CA_Mod1.txt'
trajFile = SimIn.trajFile;

%methodsview(IO.PlanReader)

% load the EUTL trajectory plan
trajPlanLLA = IO.PlanReader(trajFile).getPlan(0);

% final position and time data
initialTrajPos = trajPlanLLA.getPos(0);
finalPos = trajPlanLLA.getLastPoint.position;
finalTime = trajPlanLLA.getLastTime;

% ENU coordinate system
projection = Util.Projection.createProjection(initialTrajPos);
trajPlanENU = Util.PlanUtil.projectPlan(trajPlanLLA, projection);
%initial3dPos = trajPlanENU.getPos(0);

% trajectory position and velocity at time 0
% pos3d = trajPlanENU.position(initialTime + 0);
% vel3d = trajPlanENU.velocity(initialTime + 0);

% compute acceleration using finite difference method
%vel3dNext = trajPlanENU.velocity(initialTime + 0 + 10);

% index to the first BGS (Begin Ground Speed) point in the trajectory
%idxFirstBGS = trajPlanENU.nextBGS(-1);
idxFirstBGS = trajPlanENU.nextBVS(-1);
idxFirstEGS = trajPlanENU.nextEGS(idxFirstBGS); % find the end of the segment

% initial position and time data at segment index (start of segment)
initialPos = trajPlanLLA.getPos(idxFirstBGS);
initialTime = trajPlanLLA.time(idxFirstBGS);

% initial position and time data at segment index (end of segment)
%initialPos = trajPlanLLA.getPos(idxFirstEGS);
%initialTime = trajPlanLLA.time(idxFirstEGS);

% beginning of groundspeed change segment
initVel = trajPlanENU.initialVelocity(idxFirstBGS);

% zero longitudinal acceleration, zero vertical acceleration
%initVel = trajPlanENU.initialVelocity(idxFirstEGS)

% initial track angle (deg) and velocity (knots)
%initialPsi = initVel.trk * r2d
% use the final velocity to determine track angle, since it's not corrrect if you use the initial velocity
initialTrack = trajPlanENU.finalVelocity(idxFirstBGS).compassAngle * r2d;
initialVelocity = initVel.gs * m2ft / kt2fps;

idxLastBGS = trajPlanENU.prevBGS(trajPlanENU.size);
finalVel = trajPlanENU.finalVelocity(idxLastBGS);
%finalPsi = finalVel.trk * r2d;
finalTrack = finalVel.compassAngle * r2d;

% setup output structure
Out.trajPlanLLA     = trajPlanLLA;
Out.trajPlanENU     = trajPlanENU;

Out.initialTime     = initialTime;
%Out.initialLat      = initialPos.lat * r2d;
%Out.initialLon      = initialPos.lon * r2d;
%Out.initialAlt      = initialPos.alt * m2ft;
%Out.initialVelocity = initialVelocity;
%Out.initialTrack    = initialTrack;

% change start time with in the Ref trajectory
time_offset = 0;
Out.time_offset = time_offset;
Out.initialLat      = trajPlanLLA.position(initialTime+time_offset).lat * r2d;
Out.initialLon      = trajPlanLLA.position(initialTime+time_offset).lon * r2d;
Out.initialAlt      = trajPlanLLA.position(initialTime+time_offset).alt * m2ft;
Out.initialTrack    = trajPlanLLA.velocity(initialTime+time_offset).compassAngle * r2d;
Out.initialVelocity = trajPlanLLA.velocity(initialTime+time_offset).gs * m2ft / kt2fps;

Out.finalTime       = finalTime;
Out.finalTrack      = finalTrack;

% Process trajectory
t1 = finalTime-(initialTime+time_offset);
Out.totalTime = t1;
refTime = (0:.1:t1)';
grav = SimIn.Environment.Earth.Gravity.g0(3);

N = length(refTime);


Out.refTime = refTime;
Out.pos_NED = zeros(N,3);
Out.vel_NED = zeros(N,3);
Out.Vtot    = zeros(N,1);
Out.gamma   = zeros(N,1);
Out.phi     = zeros(N,1);
Out.chi     = zeros(N,1);
Out.chi_dot = zeros(N,1);

TR0 = getTrajectoryData(time_offset, Out, SimIn.Units);
pos_NED_0 = [TR0.pos_x TR0.pos_y TR0.pos_z];
track0 = TR0.track;
Rz = @(x) [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1];
for ii = 1:N

%   if N-ii < 50
%       M = N-ii;
%   else 
%       M = 50;
%   end
%     
%   for jj = 1:M
%     tr(jj) = getTrajectoryData(refTime(ii+jj-1)+time_offset, Out);
%   end
%   track_avg = 1/M*sum( [ tr(:).track ] );
%   chi_dot = (track_avg - track0)/.1;
%   Out.chi(ii,:)     = track_avg;
%     track0 = track_avg;
  tr0 = getTrajectoryData(refTime(ii)+time_offset, Out, SimIn.Units);
  tr1= getTrajectoryData(refTime(ii)+time_offset+1, Out, SimIn.Units);
  
  chi_dot = tr1.track - tr0.track;
  Out.chi(ii,:)     = tr0.track;
  Out.chi_dot(ii,:) = chi_dot;
  
  Vtot = tr0.velocity;
  pos_NED = [tr0.pos_x tr0.pos_y tr0.pos_z];
  vel_NED = [tr0.vel_x tr0.vel_y tr0.vel_z];

  
  
  vel_bIc = Rz(tr0.track)*vel_NED';
  phi = atan(chi_dot*Vtot/grav);

  Out.pos_NED(ii,:) = [pos_NED(1:2)-pos_NED_0(1:2)  pos_NED(3)];
  Out.vel_NED(ii,:) = vel_NED;
  Out.Vel_bIc(ii,:) = vel_bIc;
 
  
  
  Out.phi(ii,:)     = phi;


end


% temporary fix for the desired roll angle
Out.chi_dot(1:100) = 0;
Out.chi(1:100) = Out.chi(101);
Out.initialTrack    = Out.chi(1)* r2d;

%Out.phi(Out.refTime <60) = 0;
%Out.phi(:) = 0;
%Out.phi(1) = 0;
% 
Out.chi_dot(end-120:end) = 0;
Out.chi(end-120:end) = Out.chi(end-121);
Out.initialTrack    = Out.chi(1)* r2d;

