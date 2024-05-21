% Generate_Own_Traj.m script is used to generate a set of standard full
% flight trajectories for a publicly releasable NASA Lift+Cruise vehicle 
% configuration.  These trajectories are intended for use with the GUAM
% simulation and in particular to facilitate autonomous Challenge Problems

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 3.20.2024, MJA: Initial version of script.  Created for open-source
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
% Designate performance specific limits
Max_turn_rate   = 3*pi/180; % Units in rad/sec
Max_phi         = 30*pi/180; % Max angle-of-bank, units in rad
Phi_time        = 3.5; % Time to roll from wings level to necessary aob for the turn
Min_turn_ang    = 20*pi/180; % Below use two sided roll, above use 2 one-sided rolls
grav            = 32.1740; % ft/sec^2 Earths gravity constant
pre_flag        = 1; % Flag to utilize early turn computations

% User defined/derived trajectory planning variables
num_traj        = 3000; % Specify the number of random trajectories to create
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'Data_Set_1.mat';
nom_cruise      = 105; % Nominal cruise speed in knots
max_cruise      = 119; % Max cruise speed in knots
Max_num_turns   = 4; % Maximum number of mid turns (not including initial and final turn)
Max_turn_rad    = pi/3; % Maximum turn rad for mid turns...
Max_init_turn   = 4/3*pi;

% Note trajectory generated in the NED (north, east, down) frame
init_pos        = [0; 0; 0]; % Initial positions [x,y,z] in feet
init_vel        = [0 0 0]; % Specify starting/ending velocities in ft/sec
init_2_fin_len  = nom_cruise*defUnits.knot*defUnits.nmile*10/60;
final_pos       = [init_2_fin_len; 0; 40]; % Units in ft, xpos = nom_cruise knots for 10 minutes
final_vel       = [0 0 0]; % Specify starting/ending velocities in ft/sec

% Specify Initial Approach Fix and Final Approach Fix 
IAF_rad     = -pi/6; % Orientation angle of IAF to FAF
IAF_rot     = [cos(IAF_rad) -sin(IAF_rad);sin(IAF_rad) cos(IAF_rad)];
IAF_pos     = [final_pos(1:2)-IAF_rot*[0;100*defUnits.knot*180];800];
FAF_rad     = 0; % Orientation angle of FAF to final_pos
FAF_rot     = [cos(FAF_rad) -sin(FAF_rad);sin(FAF_rad) cos(FAF_rad)];
FAF_pos     = [final_pos(1:2)-FAF_rot*[0;50*defUnits.knot*60];400];
FAF_vel     = 40*defUnits.knot;

% Compute init_pos to final_pos vector and unit vector
dir_hat     = (final_pos-init_pos)/norm(final_pos-init_pos);
dir_mag     = norm(final_pos-init_pos);
% *************************************************************************

% *************************************************************************
% Create meta_data storage
% meta_NumWayPts = zeros(num_traj, 3); % Create meta data array: [num traj, number of x/y/z waypoints]

% Generate init T/O random numbers:
init_To_type    = randi([0,1],[num_traj,1]); % Random (uniform) integer 0 or 1 (0 => vertical hover T/O, 1 => fwd vel liftoff
init_Clm_rate   = (0.25-rand([num_traj,1]))*400/60+700/60; % Uniformly distributed climb rate from 400-800 ft/min
init_Fwd_vel    = ((0.5-rand([num_traj,1]))*30+25); %  Initial fwd velocity (10-40 fps)
init_max_acc    = (0.5-rand([num_traj,1]))*2+2.5; % Longitudinal and vertical (speed acceleration max)
init_Turn_rad   = (rand([num_traj,1])-0.5)*Max_init_turn; % Random inital turn (lt or rt): 4pi/3 to -4pi/3
init_alt        = 50; % Initial climb altitude (either hover or fwd flight)

% Generate init climbing/turn random profile
man1_alt        = 2000*(0.50-rand([num_traj,1]))+3000; % Initial climb to cruise altitude (2000-4000 ft uniform)
man1_rad        = pi/2*(0.50-rand([num_traj,1])); % Initial climb out turn: +/- 90 degs
man1_vel        = (8*(rand([num_traj,1])-0.25)+nom_cruise)*defUnits.knot; % Cruise speed in kts: -2 to +6 around nom_cruise speed

% Generate intermediate climbing/turn random turns
num_mid_turns   = randi(Max_num_turns,[num_traj,1]);
mid_Turn_rad    = 2*(rand([num_traj,Max_num_turns])-0.5)*Max_turn_rad; % +/- max turn

% Create storage for wpts both smoothed and original
own_traj        = cell(num_traj,6);
own_traj_orig   = cell(num_traj,6);

% Generate random trajectories
for traj_num = 1:num_traj
    
    % Initialize the cur_traj waypoint arrays
    wptsX       = [init_pos(1) init_vel(1) 0]; % Initialize pos/vel/accel in x axis
    wptsY       = [init_pos(2) init_vel(2) 0]; % Initialize pos/vel/accel in x axis
    wptsZ       = [init_pos(3) init_vel(3) 0]; % Initialize pos/vel/accel in x axis
    time_wptsX  =  0; time_wptsY  =  0; time_wptsZ  =  0;
    xy_row_ind  = [2 2];
    z_row_ind   = 2;

    % Initialize the non-smooth cur traj waypoints
    wptsX_orig      = [init_pos(1) init_vel(1) 0]; % Initialize pos/vel/accel in x axis
    wptsY_orig      = [init_pos(2) init_vel(2) 0]; % Initialize pos/vel/accel in x axis
    wptsZ_orig      = [init_pos(3) init_vel(3) 0]; % Initialize pos/vel/accel in x axis
    time_wptsX_orig =  0; time_wptsY_orig  =  0; time_wptsZ_orig  =  0;
    xy_row_ind_orig = [2 2];
    z_row_ind_orig  = 2;

    % Perform inital T/O maneuver
    if ~init_To_type(traj_num)
        % Pure vertical takeoff (smooth traj)
        init_vert_time          = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-init_vel(3));
        delta_w                 = (init_Clm_rate(traj_num)-init_vel(3))/2*init_vert_time+init_vel(3)*init_vert_time;
        wptsZ(z_row_ind, :)     = [delta_w init_Clm_rate(traj_num) 0];
        time_wptsZ(z_row_ind)   = init_vert_time;
        wptsX(xy_row_ind(1),:)  = wptsX(xy_row_ind(1)-1,:);
        wptsY(xy_row_ind(2),:)  = wptsY(xy_row_ind(2)-1,:);
        % Compute time at which init alt is achieved
        if delta_w >= init_alt
            init_fwd_time = init_vert_time;
        else
            init_fwd_time = init_vert_time + (init_alt-delta_w)/init_Clm_rate(traj_num);
        end
        time_wptsX(xy_row_ind(1)) =  init_fwd_time; 
        time_wptsY(xy_row_ind(1)) =  init_fwd_time;
        xy_row_ind  = xy_row_ind + 1;
        z_row_ind   = z_row_ind +1;

        % Update the orig trajectory
        wptsX_orig = wptsX; wptsY_orig = wptsY; wptsZ_orig = wptsZ;
        time_wptsX_orig = time_wptsX; time_wptsY_orig = time_wptsY;
        wptsZ_orig = wptsZ; time_wptsZ_orig = time_wptsZ;
        xy_row_ind_orig  = xy_row_ind_orig + 1;
        z_row_ind_orig = z_row_ind_orig +1;

        % Now transition to forward climbing flight...
        init_time               = 3/2*init_Fwd_vel(traj_num)/init_max_acc(traj_num);
        init_posit              = init_Fwd_vel(traj_num)/2*dir_hat*init_time;
        wptsX(xy_row_ind(1),:)  = [init_posit(1) dir_hat(1)*init_Fwd_vel(traj_num) 0];
        wptsY(xy_row_ind(2),:)  = [init_posit(2) dir_hat(2)*init_Fwd_vel(traj_num) 0];
        time_wptsX(xy_row_ind(1)) = time_wptsX(xy_row_ind(1)-1)+init_time;
        time_wptsY(xy_row_ind(2)) = time_wptsY(xy_row_ind(2)-1)+init_time;
        xy_row_ind  = xy_row_ind + 1;
        %z_row_ind = z_row_ind +1;

        % Now store the orig trajectory
        wptsX_orig = wptsX; wptsY_orig = wptsY; wptsZ_orig = wptsZ;
        time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX_orig(xy_row_ind_orig(1)-1)+ 0;
        time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsX_orig(xy_row_ind_orig(1)-1)+ 0;
        wptsZ_orig = wptsZ; time_wptsZ_orig = time_wptsZ;
    else
        % Forward transition takeoff (smooth traj)
        init_time                   = 3/2*init_Fwd_vel(traj_num)/init_max_acc(traj_num);
        init_posit                  = init_Fwd_vel(traj_num)/2*dir_hat*init_time;
        wptsX(xy_row_ind(1),:)      = [init_posit(1) dir_hat(1)*init_Fwd_vel(traj_num) 0];
        wptsY(xy_row_ind(2),:)      = [init_posit(2) dir_hat(2)*init_Fwd_vel(traj_num) 0];
        time_wptsX(xy_row_ind(1))   = init_time;
        time_wptsY(xy_row_ind(2))   = init_time;
                
        init_vert_time          = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-init_vel(3));
        delta_w                 = (init_Clm_rate(traj_num)-init_vel(3))/2*init_vert_time+init_vel(3)*init_vert_time;
        wptsZ(z_row_ind, :)     = [delta_w init_Clm_rate(traj_num) 0];
        time_wptsZ(z_row_ind)   = init_vert_time;
        xy_row_ind  = xy_row_ind + 1;
        z_row_ind   = z_row_ind +1;
        
        % Update the orig trajectory
        wptsX_orig = wptsX; wptsY_orig = wptsY; wptsZ_orig = wptsZ;
        time_wptsX_orig = time_wptsX; time_wptsY_orig = time_wptsY;
        wptsZ_orig = wptsZ; time_wptsZ_orig = time_wptsZ;
        xy_row_ind_orig  = xy_row_ind_orig + 1;
        z_row_ind_orig = z_row_ind_orig +1;
    end

    % Perform initial turning climb to cruise altitude, once turn complete
    % accelerate to desired cruise speed during remainder of climb
    
    % Add straight leg (to accomodate early turning...)
    str_time = 20;
    x_pos_new               = wptsX(xy_row_ind(1)-1,1)+dir_hat(1)*init_Fwd_vel(traj_num)*str_time;
    y_pos_new               = wptsY(xy_row_ind(2)-1,1)+dir_hat(2)*init_Fwd_vel(traj_num)*str_time;
    wptsX(xy_row_ind(1),:)  = [x_pos_new dir_hat(1)*init_Fwd_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)  = [y_pos_new dir_hat(2)*init_Fwd_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1)) = time_wptsX(xy_row_ind(1)-1)+str_time;
    time_wptsY(xy_row_ind(2)) = time_wptsY(xy_row_ind(2)-1)+str_time;
    xy_row_ind  = xy_row_ind + 1;

    % Update the orig trajectory
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
    xy_row_ind_orig  = xy_row_ind_orig + 1;

    % Update X&Y with initial turn 
    time_2_turn         = abs(man1_rad(traj_num)/Max_turn_rate); % Approx time # deg/ deg/sec...
    dir_temp            = [cos(man1_rad(traj_num)) -sin((man1_rad(traj_num))) 0; sin((man1_rad(traj_num))) cos((man1_rad(traj_num))) 0; 0 0 1]*dir_hat;
    x_pos_new           = wptsX(xy_row_ind(1)-1,1)+dir_temp(1)*time_2_turn*init_Fwd_vel(traj_num);
    y_pos_new           = wptsY(xy_row_ind(2)-1,1)+dir_temp(2)*time_2_turn*init_Fwd_vel(traj_num);
    wptsX(xy_row_ind(1),:)      = [x_pos_new dir_temp(1)*init_Fwd_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)      = [y_pos_new dir_temp(2)*init_Fwd_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+time_2_turn;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+time_2_turn;
    xy_row_ind  = xy_row_ind + 1;

    % Update the orig trajectory
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
    xy_row_ind_orig  = xy_row_ind_orig + 1;

    % Now update the new trajectory with early turn points
    if init_To_type(traj_num)
        ind = [2;2];
    else 
        ind = [3;3];
    end
    [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out] = ...
    ComputePreTurn(grav, [ind;ind], wptsX, wptsY, time_wptsX, time_wptsY, ...
    Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
    wptsX = wptsX_out; wptsY = wptsY_out;
    time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
    xy_row_ind = (length(time_wptsX_out) + 1)*[1;1];

    % Update X&Y with accelerate to cruise
    t_accel = 3/2/init_max_acc(traj_num)*abs(man1_vel(traj_num)- init_Fwd_vel(traj_num));
    x_pos_new          = wptsX(xy_row_ind(1)-1,1)-1/2*(man1_vel(traj_num)-init_Fwd_vel(traj_num))*dir_temp(1)*t_accel+...
                      man1_vel(traj_num)*dir_temp(1)*t_accel;
    y_pos_new          = wptsY(xy_row_ind(2)-1,1)-1/2*(man1_vel(traj_num)-init_Fwd_vel(traj_num))*dir_temp(2)*t_accel+...
                      man1_vel(traj_num)*dir_temp(2)*t_accel;
    wptsX(xy_row_ind(1),:)      = [x_pos_new dir_temp(1)*man1_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)      = [y_pos_new dir_temp(2)*man1_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+t_accel;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+t_accel;
    xy_row_ind  = xy_row_ind + 1;

    % Update the orig trajectory
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
    xy_row_ind_orig  = xy_row_ind_orig + 1;

    % **************** Update Z with climb to altitude ********************\
    % Determine position to start transition to level off 
    level_time      = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-0);
    delta_w         = (0-init_Clm_rate(traj_num))/2*level_time+init_Clm_rate(traj_num)*level_time;
    
    % Add constant rate of climb segment
    time_2_alt      = abs((man1_alt(traj_num)-init_alt-delta_w)/init_Clm_rate(traj_num));
    wptsZ(z_row_ind, :)     = [(man1_alt(traj_num)-delta_w) init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   =  time_wptsZ(z_row_ind-1)+time_2_alt;
    z_row_ind = z_row_ind +1;

    % Update the orig waypoints
    wptsZ_orig(z_row_ind_orig,:)    = wptsZ(z_row_ind-1,:);
    time_wptsZ_orig(z_row_ind_orig) = time_wptsZ(z_row_ind-1);
    z_row_ind_orig = z_row_ind_orig +1;

    % Add transition to level flight
    wptsZ(z_row_ind, :)     = [man1_alt(traj_num) 0 0];
    time_wptsZ(z_row_ind)   =  time_wptsZ(z_row_ind-1)+level_time;
    z_row_ind = z_row_ind +1;

    % Update the orig waypoints
    wptsZ_orig(z_row_ind_orig,:)    = wptsZ(z_row_ind-1,:);
    time_wptsZ_orig(z_row_ind_orig) = time_wptsZ(z_row_ind-1);
    z_row_ind_orig = z_row_ind_orig +1;

    % Continue on present heading for some timewhile climbing to
    % cruising altitude (or already reached alt) in order to facilitate early turning
    delta_t = 30; %(time_wptsZ(z_row_ind-1)-time_wptsX(xy_row_ind(1)-1))/3;
    wptsX(xy_row_ind(1),:)      = [wptsX(xy_row_ind(1)-1,1)+dir_temp(1)*man1_vel(traj_num)*delta_t dir_temp(1)*man1_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)      = [wptsY(xy_row_ind(2)-1,1)+dir_temp(2)*man1_vel(traj_num)*delta_t dir_temp(2)*man1_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+delta_t;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+delta_t;
    xy_row_ind  = xy_row_ind + 1;

    % Update the orig trajectory
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
    xy_row_ind_orig  = xy_row_ind_orig + 1;

    % Update x&y with mid turns
    while 1
        x_len_int   = rand(num_mid_turns(traj_num)+1,1);
        x_len_norm  = x_len_int/sum(x_len_int);
        if all(x_len_norm>0.1)
            break
        end
    end
    cur_2_IAF_xvec   = (IAF_pos(1)-wptsX(xy_row_ind(1)-1,1))*x_len_norm;
    % Create and update each mid turn to the IAF
    for turn = 1:num_mid_turns(traj_num)
        % Determine ang from cur pos to IAF
        cur_pos         = [wptsX(xy_row_ind(1)-1,1);wptsY(xy_row_ind(2)-1,1)];
        cur_2_IAF_vec   = IAF_pos(1:2)-cur_pos;
        cur_2_IAF_norm  = cur_2_IAF_vec/norm(cur_2_IAF_vec);
        % Rotate by the current turn and compute/update x&Y
        if turn ~= num_mid_turns(traj_num)
            cur_turn        = mid_Turn_rad(traj_num,turn);
            cur_rot         = [cos(cur_turn) -sin(cur_turn); sin(cur_turn) cos(cur_turn)];
            cur_vec         = cur_rot*cur_2_IAF_norm;
            t_scale = cur_2_IAF_xvec(turn)/cur_vec(1);
            % Compute the new x&y positions
            x_pos_new = wptsX(xy_row_ind(1)-1,1) + cur_2_IAF_xvec(turn);
            y_pos_new = wptsY(xy_row_ind(2)-1,1) + t_scale*cur_vec(2);
            % Compute the new x&y velocities
            pos_new_vec     = [x_pos_new-wptsX(xy_row_ind(1)-1,1);y_pos_new-wptsY(xy_row_ind(2)-1,1)];
            pos_new_norm    = pos_new_vec/norm(pos_new_vec); 

            % Update the x&y wpts and time_wpts
            wptsX(xy_row_ind(1),:)      = [x_pos_new pos_new_norm(1)*man1_vel(traj_num) 0];
            wptsY(xy_row_ind(2),:)      = [y_pos_new pos_new_norm(2)*man1_vel(traj_num) 0];
            time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+norm(pos_new_vec)/man1_vel(traj_num);
            time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+norm(pos_new_vec)/man1_vel(traj_num);
            xy_row_ind= xy_row_ind + 1;

            % Update the orig trajectory
            wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
            wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
            time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
            time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
            xy_row_ind_orig  = xy_row_ind_orig + 1;

            % Now update the new trajectory with early turn points
            ind = xy_row_ind-3;
            [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out] = ...
            ComputePreTurn(grav, ind, wptsX, wptsY, time_wptsX, time_wptsY, ...
            Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
            wptsX = wptsX_out; wptsY = wptsY_out;
            time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
            xy_row_ind = (length(time_wptsX_out) + 1)*[1;1];
        else
            % Try to align the mid turn to minimize turn to final (change its sign if required
            cur_turn1        = mid_Turn_rad(traj_num,turn);
            cur_rot1         = [cos(cur_turn1) -sin(cur_turn1); sin(cur_turn1) cos(cur_turn1)];
            cur_vec1         = cur_rot1*cur_2_IAF_norm;
            t_scale1 = cur_2_IAF_xvec(turn)/cur_vec1(1);
            
            % Compute the new x&y positions
            x_pos_new1 = wptsX(xy_row_ind(1)-1,1) + cur_2_IAF_xvec(turn);
            y_pos_new1 = wptsY(xy_row_ind(2)-1,1) + t_scale1*cur_vec1(2);
            cur_vec1_2_IAF = [IAF_pos(1)-x_pos_new1;IAF_pos(2)-y_pos_new1];

            cur_turn2        = -1*mid_Turn_rad(traj_num,turn);
            cur_rot2         = [cos(cur_turn2) -sin(cur_turn2); sin(cur_turn2) cos(cur_turn2)];
            cur_vec2         = cur_rot2*cur_2_IAF_norm;
            t_scale2 = cur_2_IAF_xvec(turn)/cur_vec2(1);
            
            % Compute the new x&y positions
            x_pos_new2      = wptsX(xy_row_ind(1)-1,1) + cur_2_IAF_xvec(turn);
            y_pos_new2      = wptsY(xy_row_ind(2)-1,1) + t_scale2*cur_vec2(2);
            cur_vec2_2_IAF  = [IAF_pos(1)-x_pos_new2;IAF_pos(2)-y_pos_new2];

            Fin_vec     = FAF_pos-IAF_pos;
            if acos(Fin_vec(1:2)'*cur_vec1_2_IAF/norm(Fin_vec(1:2))/norm(cur_vec1_2_IAF)) <= ...
               acos(Fin_vec(1:2)'*cur_vec2_2_IAF/norm(Fin_vec(1:2))/norm(cur_vec2_2_IAF))
                % Use the nominal mid_Turn_rad angle
                x_pos_new = x_pos_new1;
                y_pos_new = y_pos_new1;
            else % use the negative of mid_Turn_rad angle
                x_pos_new = x_pos_new2;
                y_pos_new = y_pos_new2;
            end
            
            % Compute the new x&y velocities
            pos_new_vec     = [x_pos_new-wptsX(xy_row_ind(1)-1,1);y_pos_new-wptsY(xy_row_ind(2)-1,1)];
            pos_new_norm    = pos_new_vec/norm(pos_new_vec);
            
            % Update the x&y wpts and time_wpts
            wptsX(xy_row_ind(1),:)      = [x_pos_new pos_new_norm(1)*man1_vel(traj_num) 0];
            wptsY(xy_row_ind(2),:)      = [y_pos_new pos_new_norm(2)*man1_vel(traj_num) 0];
            time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+norm(pos_new_vec)/man1_vel(traj_num);
            time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+norm(pos_new_vec)/man1_vel(traj_num);
            xy_row_ind= xy_row_ind + 1;

            % Update the orig trajectory
            wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
            wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
            time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
            time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
            xy_row_ind_orig  = xy_row_ind_orig + 1;

            % Now update the new trajectory with early turn points
            ind = xy_row_ind-3;
            [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out] = ...
            ComputePreTurn(grav, ind, wptsX, wptsY, time_wptsX, time_wptsY, ...
            Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
            wptsX = wptsX_out; wptsY = wptsY_out;
            time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
            xy_row_ind = (length(time_wptsX_out) + 1)*[1;1];
        end
    end % End of for turn = 1:num_mid_turns(traj_num)

    % Add the IAF point
    Cur_2_IAF       = IAF_pos(1:2)-[wptsX(xy_row_ind(1)-1,1);wptsY(xy_row_ind(2)-1,1)];
    pre_IAF_time    = norm(Cur_2_IAF)/man1_vel(traj_num);
    Fin_vec_norm    = Fin_vec(1:2)/norm(Fin_vec(1:2));
    wptsX(xy_row_ind(1),:)      = [IAF_pos(1) Fin_vec_norm(1)*man1_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)      = [IAF_pos(2) Fin_vec_norm(2)*man1_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+pre_IAF_time;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+pre_IAF_time;
    xy_row_ind= xy_row_ind + 1;

    % Update the orig trajectory with the IAF
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX(xy_row_ind(1)-1); 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY(xy_row_ind(2)-1);
    xy_row_ind_orig  = xy_row_ind_orig + 1;

    % Now update the new trajectory with early turn points
    ind = xy_row_ind-3;
    [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out, num_added] = ...
    ComputePreTurn(grav, ind, wptsX, wptsY, time_wptsX, time_wptsY, ...
    Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
    wptsX = wptsX_out; wptsY = wptsY_out;
    time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
    xy_row_ind = (length(time_wptsX_out) + 1)*[1;1];

    % Temporarily add the deceleration to approach speed (decel just prior to FAF)
    t_decel     = 3/2/(-1*init_max_acc(traj_num))*(FAF_vel-man1_vel(traj_num));
    x_pos_del   = -1/2*(FAF_vel-man1_vel(traj_num))*Fin_vec_norm(1)*t_decel+FAF_vel*Fin_vec_norm(1)*t_decel;
    y_pos_del   = -1/2*(FAF_vel-man1_vel(traj_num))*Fin_vec_norm(2)*t_decel+FAF_vel*Fin_vec_norm(2)*t_decel;
    x_pos_new   = FAF_pos(1)-x_pos_del;
    y_pos_new   = FAF_pos(2)-y_pos_del;
    time_2_dec_pt               = norm([x_pos_new-IAF_pos(1);y_pos_new-IAF_pos(2)])/man1_vel(traj_num);
    wptsX(xy_row_ind(1),:)      = [x_pos_new Fin_vec_norm(1)*man1_vel(traj_num) 0];
    wptsY(xy_row_ind(2),:)      = [y_pos_new Fin_vec_norm(2)*man1_vel(traj_num) 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+time_2_dec_pt;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+time_2_dec_pt;
    xy_row_ind= xy_row_ind + 1;
        
    % Now update the new trajectory with early turn points
    ind = xy_row_ind-3;
    [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out] = ...
    ComputePreTurn(grav, ind, wptsX, wptsY, time_wptsX, time_wptsY, ...
    Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
    wptsX = wptsX_out; wptsY = wptsY_out;
    time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
    xy_row_ind = length(time_wptsX_out)*[1;1]; % Overwrite last point with the decel portion

    % Overwrite the last point with the deceleration (i.e. start decel just
    % after last early turn point heading to FAF)
    x_pos_new   = wptsX(xy_row_ind(1)-1,1) + x_pos_del;
    y_pos_new   = wptsY(xy_row_ind(2)-1,1) + y_pos_del;
    wptsX(xy_row_ind(1),:)      = [x_pos_new Fin_vec_norm(1)*FAF_vel 0];
    wptsY(xy_row_ind(2),:)      = [y_pos_new Fin_vec_norm(2)*FAF_vel 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+t_decel;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+t_decel;
    xy_row_ind= xy_row_ind + 1;

    % Update the z (altitude) with the descent from cruise to IAF altitude
    level_time1      = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-0);
    delta_w1         = (init_Clm_rate(traj_num)-0)/2*level_time1+0*level_time1;
    level_time2      = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-0);
    delta_w2         = (0-init_Clm_rate(traj_num))/2*level_time2+init_Clm_rate(traj_num)*level_time2;
    alt_delta        = (man1_alt(traj_num)-IAF_pos(3));
    descent_time     = (alt_delta-delta_w1-delta_w2)/init_Clm_rate(traj_num);
    tot_des_time     = level_time1+descent_time+level_time2;
    % Add in the start of the transition
    wptsZ(z_row_ind,:)      = [man1_alt(traj_num) 0 0];
    if (time_wptsX(xy_row_ind(1)-num_added)-tot_des_time) < time_wptsZ(z_row_ind-1)
        time_wptsZ(z_row_ind) = time_wptsZ(z_row_ind-1)+10; % Delay to ensure monotonicity
    else
        time_wptsZ(z_row_ind)   = time_wptsX(xy_row_ind(1)-num_added)-tot_des_time;
    end
    z_row_ind               = z_row_ind +1;
    % Add in the stabilized descent
    wptsZ(z_row_ind,:)      = [man1_alt(traj_num)-delta_w1 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time1;
    z_row_ind               = z_row_ind +1;
    % Add in the start of the level off transition
    wptsZ(z_row_ind,:)      = [IAF_pos(3)+delta_w2 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+descent_time;
    z_row_ind               = z_row_ind +1;
    % Add in the end of the level off transition
    wptsZ(z_row_ind,:)      = [IAF_pos(3) 0 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time2;
    z_row_ind               = z_row_ind +1;

    % Update the original z (altitude) with the descent from cruise to IAF altitude
    % Add in the start of the descent to IAF
    orig_IAF_2_FAF_time             = (man1_alt(traj_num)-IAF_pos(3))/init_Clm_rate(traj_num);
    wptsZ_orig(z_row_ind_orig,:)    = [man1_alt(traj_num) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1)-1)-orig_IAF_2_FAF_time;
    z_row_ind_orig                  = z_row_ind_orig + 1;
    % Now add in the altitude at the IAF
    wptsZ_orig(z_row_ind_orig,:)    =[IAF_pos(3) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1)-1);
    z_row_ind_orig                  = z_row_ind_orig + 1;

    % Now add in the FAF with the time from the end of the decel to the FAF
    time_2_FAF = norm([FAF_pos(1); FAF_pos(2)]-[wptsX(xy_row_ind(1)-1,1);...
        wptsY(xy_row_ind(2)-1,1)])/FAF_vel;
    wptsX(xy_row_ind(1),:)      = [FAF_pos(1) Fin_vec_norm(1)*FAF_vel 0];
    wptsY(xy_row_ind(2),:)      = [FAF_pos(2) Fin_vec_norm(2)*FAF_vel 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+time_2_FAF;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+time_2_FAF;
    xy_row_ind= xy_row_ind + 1;

    % Update the orig trajectory with the FAF (modify times from IAF to FAF)
    wptsX_orig(xy_row_ind_orig(1),:) = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:) = wptsY(xy_row_ind(2)-1,:);
    orig_time_IAF_2_FAF = norm(FAF_pos(1:2)-IAF_pos(1:2))/abs(man1_vel(traj_num)-FAF_vel);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX_orig(xy_row_ind_orig(1)-1)+orig_time_IAF_2_FAF; 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY_orig(xy_row_ind_orig(2)-1)+orig_time_IAF_2_FAF;
    xy_row_ind_orig  = xy_row_ind_orig + 1;
    
    % Now add the final position
    time_2_final_pos    = norm([final_pos(1);final_pos(2)]-[FAF_pos(1); FAF_pos(2)])/FAF_vel;
    FAF_2_fin_pos_vec   = final_pos-FAF_pos;
    FAF_2_fin_pos_norm          = FAF_2_fin_pos_vec(1:2)/norm(FAF_2_fin_pos_vec(1:2));
    wptsX(xy_row_ind(1),:)      = [final_pos(1) FAF_2_fin_pos_norm(1)*FAF_vel 0];
    wptsY(xy_row_ind(2),:)      = [final_pos(2) FAF_2_fin_pos_norm(2)*FAF_vel 0];
    time_wptsX(xy_row_ind(1))   = time_wptsX(xy_row_ind(1)-1)+time_2_final_pos;
    time_wptsY(xy_row_ind(2))   = time_wptsY(xy_row_ind(2)-1)+time_2_final_pos;
    xy_row_ind= xy_row_ind + 1;

    % Update the orig trajectory with the final_pos
    wptsX_orig(xy_row_ind_orig(1),:)    = wptsX(xy_row_ind(1)-1,:); 
    wptsY_orig(xy_row_ind_orig(2),:)    = wptsY(xy_row_ind(2)-1,:);
    time_wptsX_orig(xy_row_ind_orig(1)) = time_wptsX_orig(xy_row_ind_orig(1)-1) + time_2_final_pos; 
    time_wptsY_orig(xy_row_ind_orig(2)) = time_wptsY_orig(xy_row_ind_orig(2)-1) + time_2_final_pos;
    %xy_row_ind_orig  = xy_row_ind_orig + 1;

    % Now update the new trajectory with early turn points
    ind = xy_row_ind-3;
    [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out, num_added] = ...
    ComputePreTurn(grav, ind, wptsX, wptsY, time_wptsX, time_wptsY, ...
    Max_turn_rate, Max_phi, Phi_time, Min_turn_ang, pre_flag);
    wptsX = wptsX_out; wptsY = wptsY_out;
    time_wptsX = time_wptsX_out; time_wptsY = time_wptsY_out;
    xy_row_ind(:) = length(time_wptsX);

    % Now update the altitude for the orig trajectory
    % Add in the start of the descent to IAF
    level_time1         = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-0);
    delta_w1            = (init_Clm_rate(traj_num)-0)/2*level_time1+0*level_time1;
    level_time2         = 3/2/init_max_acc(traj_num)*abs(init_Clm_rate(traj_num)-0);
    delta_w2            = (0-init_Clm_rate(traj_num))/2*level_time2+init_Clm_rate(traj_num)*level_time2;
    alt_delta_IAF_2_FAF     = (IAF_pos(3)-FAF_pos(3));
    descent_time_IAF_2_FAF  = (alt_delta_IAF_2_FAF-delta_w1-delta_w2)/init_Clm_rate(traj_num);
    alt_delta_FAF_2_fin     = (FAF_pos(3)-final_pos(3));
    descent_time_FAF_2_fin  = (alt_delta_FAF_2_fin-delta_w1-delta_w2)/init_Clm_rate(traj_num);
    % Update the IAF to FAF altitude profiles
    % Add in the start of the transition
    wptsZ(z_row_ind,:)      = [IAF_pos(3) 0 0];
    time_wptsZ(z_row_ind)   = time_wptsX(xy_row_ind(1)-num_added)-(level_time1+level_time2+descent_time_IAF_2_FAF);
    z_row_ind               = z_row_ind +1;
    % Add in the stabilized descent
    wptsZ(z_row_ind,:)      = [IAF_pos(3)-delta_w1 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time1;
    z_row_ind               = z_row_ind +1;
    % Add in the start of the level off transition
    wptsZ(z_row_ind,:)      = [FAF_pos(3)+delta_w2 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+descent_time_IAF_2_FAF;
    z_row_ind               = z_row_ind +1;
    % Add in the end of the level off transition
    wptsZ(z_row_ind,:)      = [FAF_pos(3) 0 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time2;
    z_row_ind               = z_row_ind +1;

    % Update the FAF to fin_pos altitude profiles
    wptsZ(z_row_ind,:)      = [FAF_pos(3) 0 0];
    time_wptsZ(z_row_ind)   = time_wptsX(xy_row_ind(1))-(level_time1+level_time2+descent_time_FAF_2_fin);
    z_row_ind               = z_row_ind +1;
    % Add in the stabilized descent
    wptsZ(z_row_ind,:)      = [FAF_pos(3)-delta_w1 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time1;
    z_row_ind               = z_row_ind +1;
    % Add in the start of the level off transition
    wptsZ(z_row_ind,:)      = [final_pos(3)+delta_w2 -init_Clm_rate(traj_num) 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+descent_time_FAF_2_fin;
    z_row_ind               = z_row_ind +1;
    % Add in the end of the level off transition
    wptsZ(z_row_ind,:)      = [final_pos(3) 0 0];
    time_wptsZ(z_row_ind)   = time_wptsZ(z_row_ind-1)+level_time2;

    % Now update the altitude for the orig trajectory
    % Add in the start of the descent to IAF
    orig_IAF_2_FAF_time             = (IAF_pos(3)-FAF_pos(3))/init_Clm_rate(traj_num);
    wptsZ_orig(z_row_ind_orig,:)    = [IAF_pos(3) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1)-1)-orig_IAF_2_FAF_time;
    z_row_ind_orig                  = z_row_ind_orig + 1;
    % Now add in the altitude at the FAF
    wptsZ_orig(z_row_ind_orig,:)    = [FAF_pos(3) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1)-1);
    z_row_ind_orig                  = z_row_ind_orig + 1;
    % Add in the start of the descent to fin_pos
    orig_IAF_2_FAF_time             = (FAF_pos(3)-final_pos(3))/init_Clm_rate(traj_num);
    wptsZ_orig(z_row_ind_orig,:)    = [FAF_pos(3) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1))-orig_IAF_2_FAF_time;
    z_row_ind_orig                  = z_row_ind_orig + 1;
    % Now add in the altitude at the FAF
    wptsZ_orig(z_row_ind_orig,:)    = [final_pos(3) 0 0];
    time_wptsZ_orig(z_row_ind_orig) = time_wptsX_orig(xy_row_ind_orig(1));

    % Convert the North-East-Up to North-East-Down
    wptsZ       = -1*wptsZ;
    wptsZ_orig  = -1*wptsZ_orig;
    
    % Store the results
    own_traj{traj_num,1} = wptsX; own_traj{traj_num,2} = wptsY; own_traj{traj_num,3} = wptsZ;
    own_traj{traj_num,4} = time_wptsX; own_traj{traj_num,5} = time_wptsY; own_traj{traj_num,6} = time_wptsZ;
    
    % Store the original trajectory results
    own_traj_orig{traj_num,1} = wptsX_orig; own_traj_orig{traj_num,2} = wptsY_orig; own_traj_orig{traj_num,3} = wptsZ_orig;
    own_traj_orig{traj_num,4} = time_wptsX_orig; own_traj_orig{traj_num,5} = time_wptsY_orig; own_traj_orig{traj_num,6} = time_wptsZ_orig;

    % Check to make sure that time vectors are monotonically increasing
    if any(diff(time_wptsX)<0) || any(diff(time_wptsY)<0) || any(diff(time_wptsZ)<0)
        fprintf(1,'A time vector is not monotonically increasing (invalid trajectory)\n');
        keyboard
    end
    if any(diff(time_wptsX_orig)<0) || any(diff(time_wptsY_orig)<0) || any(diff(time_wptsZ_orig)<0)
        fprintf(1,'An original time vector is not monotonically increasing (invalid trajectory)\n');
        keyboard
    end    
    
    % Plot the results...
    %Plot_PW_Bezier;
    disp('');
end % End of for traj_num...

% Output the results
fullname = fullfile(out_path, out_fname);
save(fullname,'own_traj','own_traj_orig','-v7.3');
fprintf(1,'Successfully created own-ship trajectory file (with %i trajectories):\n%s\n', num_traj, fullname);
return