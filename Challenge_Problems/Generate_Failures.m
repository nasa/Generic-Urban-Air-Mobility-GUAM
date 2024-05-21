% Generate_Failures.m script is used to generate a random set of effector 
% and propulsor failures for use with the flight trajectories for a publicly 
% releasable NASA Lift+Cruise vehicle configuration.  These failure cases 
% are intended for use with the GUAM simulation and in particular to 
% facilitate autonomous Challenge Problems
% 
% %% Define Failure Parameters
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Surface allocation
%                            
%                          / \
%                          | |  
%                 ,-------------------,
%                 '-1---------------2-'
%                          | |  
%                          | |
%                       ,---|---,
%                       '3--|--4'
%                           5 
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Failure Types: 1=Hold Last, 2=Pre-Scale, 3=Post-Scale, 4=Pos Limits
% 5=Rate Limits, 6=Generic, 7=Runaway(currently inactive), 8=Control Reversal
% SimPar.Value.Fail.Surfaces.FailInit           = [0; 0; 0; 0; 0]; % Specify failure type (see above)
% SimPar.Value.Fail.Surfaces.InitTime           = [0; 0; 0; 0; 0]; % Specify failure start time (sim time)
% SimPar.Value.Fail.Surfaces.StopTime           = [0; 0; 0; 0; 0]; % Specify failure stop time (use inf for end of sim)
% SimPar.Value.Fail.Surfaces.PreScale           = [0; 0; 0; 0; 0]; % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
% SimPar.Value.Fail.Surfaces.PostScale          = [0; 0; 0; 0; 0]; % Specify decimal post-scale factor (e.g. 0.10 or 1.3)
% SimPar.Value.Fail.Surfaces.PosBias            = zeros(SimIn.Act.nCS,1); % Specify position bias in first-order model
% SimPar.Value.Fail.Surfaces.PosScale           = zeros(SimIn.Act.nCS,1); % Specify position scale factor in first-order model
% SimPar.Value.Fail.Surfaces.UpPlim             = SimPar.Value.Actuator.MaxPos; % Specify effector upper position limit
% SimPar.Value.Fail.Surfaces.LwrPlim            = SimPar.Value.Actuator.MinPos; % Specify effector lower position limit
% SimPar.Value.Fail.Surfaces.RateBias           = zeros(SimIn.Act.nCS,1); % Specify rate bias in first-order model
% SimPar.Value.Fail.Surfaces.RateScale          = zeros(SimIn.Act.nCS,1); % Specify rate scale factor in first-order model
% SimPar.Value.Fail.Surfaces.UpRlim             = SimPar.Value.Actuator.RL; % Specify effector upper rate limit
% SimPar.Value.Fail.Surfaces.LwrRlim            = -SimPar.Value.Actuator.RL; % Specify effector lower rate limit
% SimPar.Value.Fail.Surfaces.AccelBias          = zeros(SimIn.Act.nCS,1); % Specify acceleration bias in second order model (placeholder not yet implemented)
% SimPar.Value.Fail.Surfaces.AccelScale         = zeros(SimIn.Act.nCS,1); % Specify acceleration scale factor in second order model (placeholder not yet implemented)
% SimPar.Value.Fail.Surfaces.UpAlim             = zeros(SimIn.Act.nCS,1); % Specify acceleration upper limit (placeholder not yet implemented) 
% SimPar.Value.Fail.Surfaces.LwrAlim            = zeros(SimIn.Act.nCS,1); % Specify acceleration lower limit (placeholder not yet implemented) 
% SimPar.Value.Fail.Surfaces.Generic_Sig_Select = [zeros(3,1);ones(8,1);zeros(4,1)]; % Specifies which SimPar.Fail parameters to use for generic failure (FailInit == 6), 
% Columns 1-15 are: Hold_Last, Pre-Scale, Post-Scale, Pos-Bias, Pos-Scale, Upr-Plim, Lwr-Plim,
% Rate-Bias, Rate-Scale, Upr-RateLim, Lwr-RateLim, Accel-Bias, Accel-Scale, Upr-AccelLim, Lwr-AccelLim 
%                                        
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propellers are numbered from left to right, then front to back
% 
%                 
%                          / \
%                  (1) (2) | | (3) (4)
%                 ,-------------------,
%                 '-------------------'
%                  (5) (6) | | (7) (8)
%                          | |
%                       ,-------,
%                       '-------'
%                          (9)
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Failure Types: 1=Hold Last, 2=Pre-Scale, 3=Post-Scale, 4=Pos Limits
% 5=Rate Limits, 6=Generic, 7=Runaway(currently inactive), 8 = Control Reversal
% SimPar.Value.Fail.Props.FailInit           = [fail1; 0; 0; fail4; 0; 0; 0; fail8; 0]; % Specify failure type (see above)
% SimPar.Value.Fail.Props.InitTime           = [fail_time1; 0; 0; fail_time4; 0; 0; 0; fail_time8; 0]; % Specify failure start time (sim time)
% SimPar.Value.Fail.Props.StopTime           = [1000; 0; 0; 1000; 0; 0; 0; 1000; 0]; % Specify failure stop time (use inf for end of sim)
% SimPar.Value.Fail.Props.PreScale           = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
% SimPar.Value.Fail.Props.PostScale          = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % Specify decimal post-scale factor (e.g. 0.10 or 1.3)
% SimPar.Value.Fail.Props.PosBias            = zeros(SimIn.Eng.nENG,1); % Specify position bias in first-order model
% SimPar.Value.Fail.Props.PosScale           = zeros(SimIn.Eng.nENG,1); % Specify position scale factor in first-order model
% SimPar.Value.Fail.Props.UpPlim             = SimPar.Value.Engine.MaxPos; % Specify effector upper position limit
% SimPar.Value.Fail.Props.LwrPlim            = SimPar.Value.Engine.MinPos; % Specify effector lower position limit
% SimPar.Value.Fail.Props.RateBias           = zeros(SimIn.Eng.nENG,1); % Specify rate bias in first-order model
% SimPar.Value.Fail.Props.RateScale          = zeros(SimIn.Eng.nENG,1); % Specify rate scale factor in first-order model
% SimPar.Value.Fail.Props.UpRlim             = SimPar.Value.Engine.RL; % Specify effector upper rate limit
% SimPar.Value.Fail.Props.LwrRlim            = -SimPar.Value.Engine.RL; % Specify effector lower rate limit
% SimPar.Value.Fail.Props.AccelBias          = zeros(SimIn.Eng.nENG,1); % Specify acceleration bias in second order model (placeholder not yet implemented)
% SimPar.Value.Fail.Props.AccelScale         = zeros(SimIn.Eng.nENG,1); % Specify acceleration scale factor in second order model (placeholder not yet implemented)
% SimPar.Value.Fail.Props.UpAlim             = zeros(SimIn.Eng.nENG,1); % Specify acceleration upper limit (placeholder not yet implemented) 
% SimPar.Value.Fail.Props.LwrAlim            = zeros(SimIn.Eng.nENG,1); % Specify acceleration lower limit (placeholder not yet implemented) 
% SimPar.Value.Fail.Props.Generic_Sig_Select = [zeros(3,1);ones(8,1);zeros(4,1)]; % Specifies which SimPar.Fail parameters to use for generic failure (FailInit == 6),
% Columns 1-15 are: Hold_Last, Pre-Scale, Post-Scale, Pos-Bias, Pos-Scale, Upr-Plim, Lwr-Plim,
% Rate-Bias, Rate-Scale, Upr-RateLim, Lwr-RateLim, Accel-Bias, Accel-Scale, Upr-AccelLim, Lwr-AccelLim 
% *************************************************************************

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
rng(2); % Use the matlab default: seed number = 0,  generator = Mersenne Twister

% Initialize the default units structure
defUnits = setUnits('ft','slug');
% *************************************************************************

% Specify user input parameters
own_ship_traj_file  = './Data_Set_1.mat'; % Own-ship trajectory file

% Specify user output parameters
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'Data_Set_4.mat'; % Filename of stationary obstacle data file

% Specify time intervals for which object creation is prohibited (units in seconds)
lwr_fail_time        = 10; % object creation prohibited during 0 to lwr_obj_time in own-trajectory file
upr_fail_time        = 30; % object creation prohibited during upr_obj_time to end in own-trajectory file

% Create matfile object for own-ship trajectory file
file_obj = matfile(own_ship_traj_file);
num_traj = size(file_obj.own_traj,1);

% Generate the random failure time, failure duration, failure type, failure magnitude.
fail_time_scale     = rand(num_traj,1); % This rand number will scale the failure initiation time between lwr and upr fail times
fail_dur_flag       = rand(num_traj,1); % This is used to scale the fail duration and denote use of fail time = inf
min_fail_dur        = 3; % Specify minimum length (in secs) of failure duration
max_fail_dur        = 100; % Denotes maximum scaled fail duration in secs
effector_type       = randi([0,1],[num_traj,1]); % 0=>fail propulsor, 1=>fail aero effector
fail_type           = rand(num_traj,1); % Random used to specify failure type
perc_avail          = rand(num_traj,1); % Percentage effector performance remaining (applies only to certain failure types)
min_effec_perc      = 30; % Floor of minimum effector percent performance remaining after failure (applies only to certain failure types)
max_effec_perc      = 98; % Ceiling of maximum effector percent performance remaining after failure (applies only to certain failure types)
prop_fail_ind       = randi([1,9],num_traj,1); % Random variable of failed propulsor (see figure above)
surf_fail_ind       = randi([1,5],num_traj,1); % Random variable of failed aerodynamic surface (see figure above)
num_surf            = 5; % Specify number of aero surfaces
num_prop            = 9; % Specify number of propulsors

% Create storage arrays
% own_obj: meta data storage of own-ship trajectory information at initiation of failure
own_obj                 = zeros(11,num_traj);
% storage of failure scenario specific data
Surf_FailInit_Array     = zeros(num_surf,num_traj);
Prop_FailInit_Array     = zeros(num_prop,num_traj);
Surf_InitTime_Array     = zeros(num_surf,num_traj);
Surf_StopTime_Array     = zeros(num_surf,num_traj);
Prop_InitTime_Array     = zeros(num_prop,num_traj);
Prop_StopTime_Array     = zeros(num_prop,num_traj);
Surf_PreScale_Array     = zeros(num_surf,num_traj);
Surf_PostScale_Array    = zeros(num_surf,num_traj);
Prop_PreScale_Array     = zeros(num_prop,num_traj);
Prop_PostScale_Array    = zeros(num_prop,num_traj);

for traj_num = 1:num_traj
    cur_own_traj_cell = file_obj.own_traj(traj_num,:);
    
    % Determine failure time and duration
    traj_max_time   = cur_own_traj_cell{4}(end);
    fail_time_range = traj_max_time-upr_fail_time-lwr_fail_time;
    cur_fail_time   = fail_time_scale(traj_num)*fail_time_range+lwr_fail_time;
    % compute scaled failure duration
    if fail_dur_flag(traj_num) > 0.5
        end_fail_time    = traj_max_time; % set fail duration to inf (in secs)
    else
        % set fail duration uniformly scaled duration between min and max fail duration
        end_fail_time    = cur_fail_time+2*fail_dur_flag(traj_num)*(max_fail_dur-min_fail_dur) + min_fail_dur;
    end
    % Initialize time vectors to zero
    Surf_InitTime = zeros(num_surf,1);
    Surf_StopTime = zeros(num_surf,1);
    Prop_InitTime = zeros(num_prop,1);
    Prop_StopTime = zeros(num_prop,1);
    if effector_type(traj_num) 
        % Surface Failure
        Surf_InitTime(surf_fail_ind(traj_num)) = cur_fail_time;
        Surf_StopTime(surf_fail_ind(traj_num)) = end_fail_time;
    else 
        % Propulsor Failure
        Prop_InitTime(prop_fail_ind(traj_num)) = cur_fail_time;
        Prop_StopTime(prop_fail_ind(traj_num)) = end_fail_time;
    end

    % Compute scaled failure percentage.  Note this finds the percentage of
    % effector effectiveness REMAINING not the percent FAILED.
    perc_eff   = perc_avail(traj_num)*(max_effec_perc-min_effec_perc)+min_effec_perc;

    % Determine specific failure type 
    if fail_type(traj_num) >=0 && fail_type(traj_num)< 0.05
            fail_type_act = 1; % Hold last failure
    elseif fail_type(traj_num) >= 0.05 && fail_type(traj_num) < 0.54
            fail_type_act = 2; % Pre-scale failure, (failure in command prior to actuator dynamics)
    elseif fail_type(traj_num) >= 0.54 && fail_type(traj_num) < 0.99
            fail_type_act = 3; % Post-scale failure, (failure after actuator dynamics)
    elseif fail_type(traj_num) >= 0.99 && fail_type(traj_num) <= 1.0
            fail_type_act = 8; % Control reversal (applies only to aero effectors)
    end

    Surf_FailInit   = zeros(num_surf,1);
    Surf_PreScale   = zeros(num_surf,1);
    Surf_PostScale  = zeros(num_surf,1);
    Prop_FailInit   = zeros(num_prop,1);
    Prop_PreScale   = zeros(num_prop,1);
    Prop_PostScale  = zeros(num_prop,1);

    if ~effector_type(traj_num) && fail_type_act ~=8 % propulsor failure and NOT control reversal
        % Propulsor Failure
        Prop_FailInit(prop_fail_ind(traj_num)) = fail_type_act;
        if fail_type_act == 2 % pre scale
            Prop_PreScale(prop_fail_ind(traj_num)) = perc_eff;
        end
        if fail_type_act == 3 % post scale
            Prop_PostScale(prop_fail_ind(traj_num)) = perc_eff;
        end
    else
        % Surface Failure
        Surf_FailInit(surf_fail_ind(traj_num)) = fail_type_act;
        if fail_type_act == 2 % pre scale
            Surf_PreScale(surf_fail_ind(traj_num)) = perc_eff;
        end
        if fail_type_act == 3 % post scale
            Surf_PostScale(surf_fail_ind(traj_num)) = perc_eff;
        end
    end
    
    % Compute position and velocity of own-ship traj at cur_fail_time
    cur_pwcurve=genPWCurve({cur_own_traj_cell{1},cur_own_traj_cell{2},cur_own_traj_cell{3}},...
        {cur_own_traj_cell{4}, cur_own_traj_cell{5}, cur_own_traj_cell{6}});
    cur_pos     = evalPWCurve(cur_pwcurve,cur_fail_time,0);
    cur_vel     = evalPWCurve(cur_pwcurve,cur_fail_time,1);
    cur_accel   = evalPWCurve(cur_pwcurve, cur_fail_time,2);
  
    % Store the results
    % own_obj = zeros(11,num_traj); % storage: traj_num, cur_fail_time, cur_pos, cur_vel, cur_accel
    own_obj(:,traj_num) =  [traj_num;cur_fail_time; cur_pos'; cur_vel'; cur_accel'];
    
    Surf_FailInit_Array(:,traj_num)     = Surf_FailInit;
    Prop_FailInit_Array(:,traj_num)     = Prop_FailInit;
    Surf_InitTime_Array(:,traj_num)     = Surf_InitTime;
    Surf_StopTime_Array(:,traj_num)     = Surf_StopTime;
    Prop_InitTime_Array(:,traj_num)     = Prop_InitTime;
    Prop_StopTime_Array(:,traj_num)     = Prop_StopTime;
    Surf_PreScale_Array(:,traj_num)     = Surf_PreScale;
    Surf_PostScale_Array(:,traj_num)    = Surf_PostScale;
    Prop_PreScale_Array(:,traj_num)     = Prop_PreScale;
    Prop_PostScale_Array(:,traj_num)    = Prop_PostScale;

    disp('');
end

% Output the results
fullname = fullfile(out_path, out_fname);
save(fullname,'own_obj', 'Surf_FailInit_Array', 'Prop_FailInit_Array', ...
    'Surf_InitTime_Array', 'Surf_StopTime_Array', 'Surf_StopTime_Array', ...
    'Prop_InitTime_Array', 'Prop_StopTime_Array', 'Surf_PreScale_Array', ...
    'Surf_PostScale_Array', 'Prop_PreScale_Array', 'Prop_PostScale_Array', ...
    '-v7.3');
fprintf(1,'Successfully created effector failure file (with %i trajectories):\n%s\n', num_traj, fullname);

% *********** Delete below here *******************************************
% cur_vel_hat = cur_vel'/norm(cur_vel);

%     % Create vector perpendicular to velocity with z component directly
%     % above the trajectory point
%     temp_vec            = [0;0;-1];     
%     temp_vec_perp       = temp_vec-(temp_vec'*cur_vel_hat)*cur_vel_hat;
%     temp_vec_perp_hat   = temp_vec_perp/norm(temp_vec_perp);
% 
%     % Create the quaternion to rotate around the velocity vector hat
%     qr = [cos(obj_orient(traj_num)/2);sin(obj_orient(traj_num)/2)*cur_vel_hat];
% 
%     % Rotate the perp unit vector around the unit velocity vector
%     temp_perp_hat_rot = Qtrans(qr, temp_vec_perp_hat);
% 
%     % Determine the position of the stationary object center
%     obj_cent_pos = cur_pos'+temp_perp_hat_rot*obj_perp_dist(traj_num);