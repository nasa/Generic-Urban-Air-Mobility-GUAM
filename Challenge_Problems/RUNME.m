% RUNME.m script is used to demonstrate use of some of the Challenge Problem
% data sets with the GUAM simulation.  In particular, this file demonstrates
% setting up the GUAM simulation to use one of the own-ship dynamically 
% feasible trajectories from Data_Set_1.m in conjunction with one of the 
% failure cases from Data_Set_4.m for the publicly  releasable NASA Lift+Cruise 
% vehicle configuration. This file should be run from the
% Challenge_Problems/ folder not the main directory.

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 5.2.2024, MJA: Initial version of script.  Created for open-source
% release with GUAM version 1.1
% *************************************************************************

userStruct.variants.refInputType = 4; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
userStruct.variants.fmType      = 2; % 1 = Aerodynamic model, 1 = s-function, 2 = polynomial
userStruct.variants.propType    = 4; % Select- propulsor model, 1=None, 2=First Order, 3=Second Order, 4 =First order fail
userStruct.variants.actType     = 4; % Select aero effector model, 1=None, 2=First Order, 3=Second Order, 4 =First order fail

file_obj = matfile('./Data_Set_1.mat'); % Specify the own-ship trajectory file
traj_run_num = 3; % Select the desired own-ship trajectory number
fail_run_num = 3; % Select the corresponding failure case number (Note this could differ from the desired traj number)
% NOTE: the traj/failure run case (#3) demonstrated here causes the GUAM
% simulation to fail as it departs flight ~5 secs after failure initiation
% leading to the aircraft states exceeded those defined for the Polynomial
% model (see /vehicles/Lift+Cruise/AeroProp/Polynomial/LpC_interp_p_v2.m)
model = 'GUAM';

% Define the own-ship Bernstein trajectory
wptsX_cell = file_obj.own_traj(traj_run_num,1);
wptsY_cell = file_obj.own_traj(traj_run_num,2);
wptsZ_cell = file_obj.own_traj(traj_run_num,3);
time_wptsX_cell = file_obj.own_traj(traj_run_num,4);
time_wptsY_cell = file_obj.own_traj(traj_run_num,5);
time_wptsZ_cell = file_obj.own_traj(traj_run_num,6);

target.RefInput.Bezier.waypoints = {wptsX_cell{1}, wptsY_cell{1}, wptsZ_cell{1}};
target.RefInput.Bezier.time_wpts = {time_wptsX_cell{1}, time_wptsY_cell{1}, time_wptsZ_cell{1}};

% Set desired GUAM Initial condition vars
target.RefInput.Vel_bIc_des    = [wptsX_cell{1}(1,2) wptsY_cell{1}(1,2) wptsZ_cell{1}(1,2)];
target.RefInput.pos_des        = [wptsX_cell{1}(1,1) wptsY_cell{1}(1,1) wptsZ_cell{1}(1,1)];
target.RefInput.chi_des        = atan2(wptsY_cell{1}(1,2),wptsX_cell{1}(1,2));
target.RefInput.chi_dot_des    = 0;
target.RefInput.trajectory.refTime = [0 time_wptsX_cell{1}(end)];

% Run the GUAM simSetup script
cd('../');
simSetup % Run the model setup scripts

% Implement failure scenario if desired.
if 1 % Change to 0/1 to add failure scenario as desired
    % Load the failure scenario Challenge Problem data sets
    fail_obj = matfile('./Challenge_Problems/Data_Set_4.mat');

    % Set GUAM SimPar failure parameters
    SimPar.Value.Fail.Surfaces.FailInit     = fail_obj.Surf_FailInit_Array(:, fail_run_num);% Specify failure type (see above)
    SimPar.Value.Fail.Surfaces.InitTime     = fail_obj.Surf_InitTime_Array(:, fail_run_num); % Specify failure start time (sim time)
    SimPar.Value.Fail.Surfaces.StopTime     = fail_obj.Surf_StopTime_Array(:, fail_run_num); % Specify failure stop time (use inf for end of sim)
    SimPar.Value.Fail.Surfaces.PreScale     = fail_obj.Surf_PreScale_Array(:, fail_run_num); % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
    SimPar.Value.Fail.Surfaces.PostScale    = fail_obj.Surf_PostScale_Array(:, fail_run_num); % Specify decimal post-scale factor (e.g. 0.10 or 1.3)

    SimPar.Value.Fail.Props.FailInit    = fail_obj.Prop_FailInit_Array(:, fail_run_num); % Specify failure type (see above)
    SimPar.Value.Fail.Props.InitTime    = fail_obj.Prop_InitTime_Array(:, fail_run_num); % Specify failure start time (sim time)
    SimPar.Value.Fail.Props.StopTime    = fail_obj.Prop_StopTime_Array(:, fail_run_num); % Specify failure stop time (use inf for end of sim)
    SimPar.Value.Fail.Props.PreScale    = fail_obj.Prop_PreScale_Array(:, fail_run_num); % Specify decimal pre-scale factor (e.g. 0.10 or 1.3)
    SimPar.Value.Fail.Props.PostScale   = fail_obj.Prop_PostScale_Array(:, fail_run_num); % Specify decimal post-scale factor (e.g. 0.10 or 1.3)
end

% Open the GUAM model 
open(model)