% Plot_Chal_Prob_DSets.m script is used to plot user selected portions of the 
% Challenge problem Data Sets (own-ship trajectory, stationary obstacles, and
% moving obstacles).These data sets are for the publicly releasable NASA 
% Lift+Cruise vehicle configuration.  These data sets are intended for use 
% with the GUAM simulation and in particular to facilitate research on
% autonomous Challenge Problems

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 5.3.2024, MJA: Initial version of script.  Created for open-source
% release with GUAM version 1.1
% *************************************************************************

% Add required paths to run this file stand-alone (not required if
% simSetup.m was run)
if ~exist('userStruct','var')
    addpath('../Challenge_Problems');
    addpath('../Bez_Functions');
    addpath('../lib/utilities');
end

% Initialize the default units structure
defUnits = setUnits('ft','slug');

% Challenge problem data sets
OwnTraj_fname       = './Data_Set_1.mat'; % Filename of own-ship trajectory data file
StatObj_fname       = './Data_Set_2.mat'; % Filename of stationary obstacle data file
MovObj_fname        = './Data_Set_3.mat'; % Filename of moving obstacle data file

% Select the desired data runs to plot from each data file.
% (Non-zero/non-empty data runs will be plotted.  Row vectors of desired
% runs are acceptable)

Desired_OwnTraj     = 1:5; %1:5; % 4:5;
Desired_StatObj     = 1:5;%1:5; % 4:5;
Desired_MovObj      = 1:5; % 4:5;

% Specify the trajectory times of interest to plot
t_start_des = []; % Empty set implies beginning of trajectory, must be >= 0
t_end_des   = []; % Empty set implies end of trajectory, must be > t_start_des
t_int       = 0.01; % Select the plotting time step

% Create the figure
DS_fig = figure; % Create handle to Data Sets figure
hold on;
grid on;
cur_ax = gca;
% *************************************************************************
% Plot the desired own-trajectory(ies)
OwnTraj_fobj = matfile(OwnTraj_fname);
leg_cell     = cell(length(Desired_OwnTraj),1); 
leg_cnt      = 0;
for loop = Desired_OwnTraj
    % Load in the data for just the desired own-ship trajectory
    cur_traj_cell = OwnTraj_fobj.own_traj(loop,:);
    % Create the pw Bernstein polynomial curve
    CurTraj_pwcurve = genPWCurve({cur_traj_cell{1},cur_traj_cell{2},cur_traj_cell{3}},...
        {cur_traj_cell{4}, cur_traj_cell{5}, cur_traj_cell{6}});
    % Plot the trajectory

    if isempty(t_start_des) 
        t_start = 0;
    else
        t_start = t_start_des;
    end
    if isempty(t_end_des) || t_end_des > cur_traj_cell{4}(end)
        t_end = cur_traj_cell{4}(end);
    else
        t_end = t_end_des;
    end
    time    = linspace(t_start, t_end, (t_end-t_start)/t_int); % Create linearly spaced time vector
    pos     = evalPWCurve(CurTraj_pwcurve, time,0);
    plot3(cur_ax, pos(:,1), pos(:,2), pos(:,3));
    hold all;

    leg_cnt              = leg_cnt + 1;
    leg_cell{leg_cnt}    = sprintf('Own-ship Traj #%i',loop);
end
% ************************************************************************

% *************************************************************************
%       Plot the desired stationary object(s)
StatObj_fobj = matfile(StatObj_fname);

for loop = Desired_StatObj
    % Load in the data for just the desired stationary obstacle
    % stat_obj: obj_cen_pos, obj_rad, obj_perp_distance, traj_time traj_pos, traj_num
    stat_obj_cell = StatObj_fobj.stat_obj(:,loop);
    
    % Create a ball of the desired radius
    [x, y, z] = sphere;
    stat_objx = x*stat_obj_cell(4)+stat_obj_cell(1);
    stat_objy = y*stat_obj_cell(4)+stat_obj_cell(2);
    stat_objz = z*stat_obj_cell(4)+stat_obj_cell(3);
    surf(cur_ax, stat_objx, stat_objy, stat_objz,'FaceAlpha',0.5);
    leg_cnt              = leg_cnt + 1;
    leg_cell{leg_cnt}    = sprintf('Stationary Obj#%i',loop);
end
% *************************************************************************

% *************************************************************************
%                  Plot the desired moving object(s)
MovObj_fobj = matfile(MovObj_fname);
for loop = Desired_MovObj
    % Load in the data for just the desired moving obstacle
    % mov_obj: obj_cen_pos, obj_rad, obj_perp_distance, traj_time, traj_pos, traj_num
    mov_obj_cell = MovObj_fobj.mov_obj(:,loop);
    
    % Create a ball of the desired radius
    [x, y, z] = sphere;
    mov_objx = x*mov_obj_cell(4)+mov_obj_cell(1);
    mov_objy = y*mov_obj_cell(4)+mov_obj_cell(2);
    mov_objz = z*mov_obj_cell(4)+mov_obj_cell(3);
    surf(cur_ax, mov_objx, mov_objy, mov_objz,'FaceAlpha',0.5);
    leg_cnt              = leg_cnt + 1;
    leg_cell{leg_cnt}    = sprintf('Moving Obj#%i Collision',loop);

    % ************** Now plot the intruder trajectory *********************
    % Load in the data for just the desired mov obs trajectory
    intr_traj_cell = MovObj_fobj.intr_traj(loop,:);
    % Create the pw Bernstein polynomial curve
    IntrTraj_pwcurve = genPWCurve({intr_traj_cell{1},intr_traj_cell{2},intr_traj_cell{3}},...
        {intr_traj_cell{4}, intr_traj_cell{5}, intr_traj_cell{6}});
    % Plot the trajectory

    if isempty(t_start) || t_start < intr_traj_cell{4}(1) || t_start > intr_traj_cell{4}(end)
        t_start = intr_traj_cell{4}(1);
    end
    if isempty(t_end) || t_end < intr_traj_cell{4}(1) || t_end > intr_traj_cell{4}(end)
        t_end = intr_traj_cell{4}(end);
    end
    time    = linspace(t_start,t_end, (t_end-t_start)/t_int); % Create linearly spaced time vector
    pos     = evalPWCurve(IntrTraj_pwcurve, time, 0);
    plot3(cur_ax, pos(:,1), pos(:,2), pos(:,3));
    hold all;
end
% *************************************************************************

xlabel('North (ft)');
ylabel('East (ft)');
zlabel('Down (ft)');
grid on;
legend(leg_cell);