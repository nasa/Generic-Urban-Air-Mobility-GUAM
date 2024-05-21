% Generate_Stat_Obst.m script is used to generate a set of stationary 
% obstacles (balls) for use with the flight trajectories for a publicly 
% releasable NASA Lift+Cruise vehicle configuration.  These obstacles 
% are intended for use with the GUAM simulation and in particular to 
% facilitate autonomous Challenge Problems

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 5.2.2024, MJA: Initial version of script.  Created for open-source
% release with GUAM version 1.1
% *************************************************************************

% Add required paths to run this file stand-alone (not required if
% simSetup.m was run)
if ~exist('userStruct','var')
    addpath('../Challenge_Problems');
    addpath('../Bez_Functions');
    addpath('../lib/utilities');
end

% Specify the seed/generator for use with random number generator
rng('default'); % Use the matlab default: seed number =0,  generator = Mersenne Twister

% Initialize the default units structure
defUnits = setUnits('ft','slug');
% *************************************************************************

% Specify user input parameters
own_ship_traj_file  = './Data_Set_1.mat';
max_size            = 1000; % Specify maximum obstacle radius in feet
min_size            = 200; % Specify minimum obstacle radius in feet
max_perp_dist       = 200; % Maximum allowed perp distance to some portion of the stationary obstacle

% Specify user output parameters
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'Data_Set_2.mat'; % Filename of stationary obstacle data file

% Specify time intervals for which object creation is prohibited (units in seconds)
lwr_obj_time        = 45; % object creation prohibited during 0 to lwr_obj_time in own-trajectory file
upr_obj_time        = 120; % object creation prohibited during upr_obj_time to end in own-trajectory file

% Create matfile object for own-ship trajectory file
file_obj = matfile(own_ship_traj_file);
num_traj = size(file_obj.own_traj,1);

% Generate the random object sizes, occurrence time, etc.
obj_time        = rand(num_traj,1); % This rand number will scale object creation time between lwr and upr trajectory times
obj_orient      = rand(num_traj,1)*2*pi; % Orientation of object ball center with respect to xy velocity vector at object time
obj_rad         = rand(num_traj,1)*(max_size-min_size)+min_size; % Object ball radius (units ft)
obj_perp_dist   = rand(num_traj,1).*(obj_rad+max_perp_dist); % Distance (ft) of object ball center from own-trajectory at obj_time

% Create storage array
stat_obj = zeros(10,num_traj); % storage: obj_cen_pos, obj_rad, obj_perp_distance, traj_time traj_pos, traj_num
for traj_num = 1:num_traj
    cur_own_traj_cell = file_obj.own_traj(traj_num,:);
    
    % Determine obj trajectory time
    traj_max_time = cur_own_traj_cell{4}(end);
    obj_time_range = traj_max_time-upr_obj_time-lwr_obj_time;
    cur_obj_time = obj_time(traj_num)*obj_time_range+lwr_obj_time;
    
    % Compute position and velocity of own-ship traj at cur_obj_time
    cur_pwcurve=genPWCurve({cur_own_traj_cell{1},cur_own_traj_cell{2},cur_own_traj_cell{3}},...
        {cur_own_traj_cell{4}, cur_own_traj_cell{5}, cur_own_traj_cell{6}});
    cur_pos     = evalPWCurve(cur_pwcurve,cur_obj_time,0);
    cur_vel     = evalPWCurve(cur_pwcurve,cur_obj_time,1);
    cur_vel_hat = cur_vel'/norm(cur_vel);

    % Create vector perpendicular to velocity with z component directly
    % above the trajectory point
    temp_vec            = [0;0;-1];     
    temp_vec_perp       = temp_vec-(temp_vec'*cur_vel_hat)*cur_vel_hat;
    temp_vec_perp_hat   = temp_vec_perp/norm(temp_vec_perp);

    % Create the quaternion to rotate around the velocity vector hat
    qr = [cos(obj_orient(traj_num)/2);sin(obj_orient(traj_num)/2)*cur_vel_hat];

    % Rotate the perp unit vector around the unit velocity vector
    temp_perp_hat_rot = Qtrans(qr, temp_vec_perp_hat);

    % Determine the position of the stationary object center
    obj_cent_pos = cur_pos'+temp_perp_hat_rot*obj_perp_dist(traj_num);

    % Store the results
    % stat_obj: obj_cen_pos, obj_rad, obj_perp_distance, traj_time traj_pos, traj_num
    stat_obj(:,traj_num) =  [obj_cent_pos; obj_rad(traj_num);obj_perp_dist(traj_num);cur_obj_time;cur_pos';traj_num];
end

% Output the results
fullname = fullfile(out_path, out_fname);
save(fullname,'stat_obj','-v7.3');
fprintf(1,'Successfully created stationary obstacle file (with %i trajectories):\n%s\n', num_traj, fullname);