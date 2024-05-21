% Generate_Mov_Obst.m script is used to generate a set of moving 
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
rng(1); % Use the matlab default: seed number =0,  generator = Mersenne Twister

% Initialize the default units structure
defUnits = setUnits('ft','slug');
% *************************************************************************

% *************************************************************************
% Specify user input parameters

% Specify own-trajectory input path/filename
own_ship_traj_file  = './Data_Set_1.mat';

% Specify point of collision obstacle ball parameters
max_size            = 1000*defUnits.ft; % Specify maximum obstacle radius in feet
min_size            = 200*defUnits.ft; % Specify minimum obstacle radius in feet
max_perp_dist       = 200*defUnits.ft; % Maximum allowed perp distance to some portion of the stationary obstacle

% Specify moving intruder parameters
max_intr_spd            = 140*defUnits.knot; % Specify upper range random intruder obstacle speed
min_intr_spd            = 60*defUnits.knot; % Specify lower range of random intruder obstacle speed
intr_alt_var            = 1000*defUnits.ft; % Specify intruder altitude variation at intruder_pre_time
intr_pre_time_mean      = 120; % Mean time which intruder trajectory is defined before "collision"
intr_post_time_mean     = 120; % Mean time which intruder trajectory is defined before "collision"
intr_time_var           = 20; % range of variation for intruder pre and post time
intr_traj_time_fac      = 0.15; % Range of variation in intruder time from nominal time (introduces curve in intruder Bernstein trajectory)
perc_rectilinear_intr   = 1; %0.5; % Specifies percentage of intruder trajs that are rectilear in x&y plane

% Specify user output filename
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'Data_Set_3.mat'; % Filename of stationary obstacle data file

% Specify time intervals for which object creation is prohibited (units in seconds)
lwr_obj_time        = 120; % object creation prohibited during 0 to lwr_obj_time in own-trajectory file
upr_obj_time        = 120; % object creation prohibited during upr_obj_time to end in own-trajectory file
% *************************************************************************

% *************************************************************************
% Create matfile object for own-ship trajectory file
file_obj = matfile(own_ship_traj_file);
num_traj = size(file_obj.own_traj,1);

% Generate the random object sizes, occurrence time, at the intruder "collision" point
obj_time        = rand(num_traj,1); % This rand number will scale object creation time between lwr and upr trajectory times
obj_orient      = rand(num_traj,1)*2*pi; % Orientation of object ball center with respect to xy velocity vector at object time
obj_rad         = rand(num_traj,1)*(max_size-min_size)+min_size; % Object ball radius (units ft)
obj_perp_dist   = rand(num_traj,1).*(obj_rad+max_perp_dist); % Distance (ft) of object ball center from own-trajectory at obj_time

% Generate random intruder mean velocities, collision orientation angles,
% collision change of altitude, and pre-collision intruder trajectory time 
% and post-collision intruder trajectory time
intr_spd            = rand(num_traj,1)*(max_intr_spd-min_intr_spd)+ min_intr_spd;
intr_orient         = rand(num_traj,1)*2*pi; % Orientation of intruder velocity at "collision"
intr_pre_time       = 2*(rand(num_traj,1)-0.5)*intr_time_var+intr_pre_time_mean;
intr_post_time      = 2*(rand(num_traj,1)-0.5)*intr_time_var+intr_post_time_mean;
intr_pre_alt_delta  = 2*(rand(num_traj,1)-0.5)*intr_alt_var;
intr_post_alt_delta = 2*(rand(num_traj,1)-0.5)*intr_alt_var;

% Create storage array for the collision point information
mov_obj = zeros(10,num_traj); % storage: obj_cen_pos, obj_rad, obj_perp_distance, traj_time traj_pos, intr_num
% Create storage for wpts of intruder trajectory
intr_traj        = cell(num_traj,6);
for intr_num = 1:num_traj
    cur_own_traj_cell = file_obj.own_traj(intr_num,:);
    
    % Determine obj trajectory time
    traj_max_time = cur_own_traj_cell{4}(end);
    obj_time_range = traj_max_time-upr_obj_time-lwr_obj_time;
    cur_obj_time = obj_time(intr_num)*obj_time_range+lwr_obj_time;
    
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
    qr = [cos(obj_orient(intr_num)/2);sin(obj_orient(intr_num)/2)*cur_vel_hat];

    % Rotate the perp unit vector around the unit velocity vector
    temp_perp_hat_rot = Qtrans(qr, temp_vec_perp_hat);

    % Determine the position of the moving object center at time of "collision"
    obj_cent_pos = cur_pos'+temp_perp_hat_rot*obj_perp_dist(intr_num);

    % Store the results
    % mov_obj_col: obj_cen_pos, obj_rad, obj_perp_distance, traj_time traj_pos, intr_num
    mov_obj(:,intr_num) =  [obj_cent_pos; obj_rad(intr_num);obj_perp_dist(intr_num);cur_obj_time;cur_pos';intr_num];

    % Now compute a trajectory for the moving intruder
    % intr_spd, intr_pre_time, intr_post_time,
    % intr_pre_alt_deltat,intr_post_alt_delta, intr_traj_time_fac, perc_rectilinear_intr
    
    % Define and intialize intruder Bernstein polynomial arrays
    wptsX_intr = zeros(3,3); wptsY_intr = zeros(3,3); wptsZ_intr = zeros(3,3);
    time_wptsX_intr = zeros(1,3); time_wptsY_intr = zeros(1,3); time_wptsZ_intr = zeros(1,3);
    
    if intr_num/num_traj > perc_rectilinear_intr
        % Non-rectilinear flight: to be written subsequently
        disp('');
    else
        %  **************** Rectilear flight profile **********************
        % Specify intruder x&y at pre-collision point
        if cur_obj_time-intr_pre_time(intr_num) > 0
            act_pre_time = cur_obj_time-intr_pre_time(intr_num);
        else
            act_pre_time = 0;
        end
        preX_pos = obj_cent_pos(1)-cos(intr_orient(intr_num))*intr_spd(intr_num)*(cur_obj_time-act_pre_time);
        preY_pos = obj_cent_pos(2)-sin(intr_orient(intr_num))*intr_spd(intr_num)*(cur_obj_time-act_pre_time);
        wptsX_intr(1,:)     = [preX_pos cos(intr_orient(intr_num))*intr_spd(intr_num) 0];
        wptsY_intr(1,:)     = [preY_pos sin(intr_orient(intr_num))*intr_spd(intr_num) 0];
        %time_wptsX_intr(1)  = norm([obj_cent_pos(1)-preX_pos; obj_cent_pos(2)-preY_pos])/intr_spd(num_traj);
        time_wptsX_intr(1)  = act_pre_time;
        time_wptsY_intr(1)  = time_wptsX_intr(1);
        if obj_cent_pos(3)-intr_pre_alt_delta(intr_num)<0
            intr_pre_alt    = obj_cent_pos(3)-intr_pre_alt_delta(intr_num);
        else 
            intr_pre_alt    = 0;
        end
        pre_vert_vel        = (obj_cent_pos(3)-intr_pre_alt)/(cur_obj_time-time_wptsX_intr(1));
        wptsZ_intr(1,:)     = [intr_pre_alt pre_vert_vel 0];
        time_wptsZ_intr(1)  = time_wptsX_intr(1);

        % Specify the intruder at the "collision" point
        wptsX_intr(2,:)     = [obj_cent_pos(1) cos(intr_orient(intr_num))*intr_spd(intr_num) 0];
        wptsY_intr(2,:)     = [obj_cent_pos(2) sin(intr_orient(intr_num))*intr_spd(intr_num) 0];
        time_wptsX_intr(2)  = cur_obj_time; time_wptsY_intr(2) = cur_obj_time;    % Specify intruder x&y at post-collision point
        wptsZ_intr(2,:)     = [obj_cent_pos(3) pre_vert_vel 0];
        time_wptsZ_intr(2)  = cur_obj_time;

%         % Specify intruder x&y at post-collision point
%         if cur_obj_time+intr_post_time(num_traj) > cur_own_traj_cell{1}(end)
%             act_post_time = cur_obj_time+intr_post_time(num_traj);
%         else
%             act_post_time = cur_own_traj_cell{1}(end);
%         end
        act_post_time = cur_obj_time+intr_post_time(intr_num);
        postX_pos = obj_cent_pos(1)+cos(intr_orient(intr_num))*intr_spd(intr_num)*(act_post_time-cur_obj_time);
        postY_pos = obj_cent_pos(2)+sin(intr_orient(intr_num))*intr_spd(intr_num)*(act_post_time-cur_obj_time);
        wptsX_intr(3,:)     = [postX_pos cos(intr_orient(intr_num))*intr_spd(intr_num) 0];
        wptsY_intr(3,:)     = [postY_pos sin(intr_orient(intr_num))*intr_spd(intr_num) 0];
        time_wptsX_intr(3)  = act_post_time;
        time_wptsY_intr(3)  = time_wptsX_intr(3);

        post_vert_vel       = pre_vert_vel;
        intr_post_alt       = obj_cent_pos(3) + post_vert_vel*(time_wptsX_intr(3)-time_wptsX_intr(2));
        wptsZ_intr(3,:)     = [intr_post_alt post_vert_vel 0];
        time_wptsZ_intr(3)  = time_wptsX_intr(3);

        % Store the results
        intr_traj{intr_num,1} = wptsX_intr; intr_traj{intr_num,2} = wptsY_intr; intr_traj{intr_num,3} = wptsZ_intr;
        intr_traj{intr_num,4} = time_wptsX_intr; intr_traj{intr_num,5} = time_wptsY_intr; intr_traj{intr_num,6} = time_wptsZ_intr;
    end
    disp('');

end

% Output the results
fullname = fullfile(out_path, out_fname);
save(fullname,'mov_obj','intr_traj','-v7.3');
fprintf(1,'Successfully created moving obstacle file (with %i trajectories):\n%s\n', num_traj, fullname);