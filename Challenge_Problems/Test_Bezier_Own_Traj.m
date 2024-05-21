% This file is used to demo loading and running an own-ship trajectory from
% the Challenge Problem own-ship data file

userStruct.variants.refInputType = 4; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
userStruct.variants.fmType = 2; % 1 = Aerodynamic model, 1 = s-function, 2 = polynomial
file_obj = matfile('./Data_Set_1.mat');
run_num = 3;
model = 'GUAM';

wptsX_cell = file_obj.own_traj(run_num,1);
wptsY_cell = file_obj.own_traj(run_num,2);
wptsZ_cell = file_obj.own_traj(run_num,3);
time_wptsX_cell = file_obj.own_traj(run_num,4);
time_wptsY_cell = file_obj.own_traj(run_num,5);
time_wptsZ_cell = file_obj.own_traj(run_num,6);

target.RefInput.Bezier.waypoints = {wptsX_cell{1}, wptsY_cell{1}, wptsZ_cell{1}};
target.RefInput.Bezier.time_wpts = {time_wptsX_cell{1}, time_wptsY_cell{1}, time_wptsZ_cell{1}};

% Set desired Initial condition vars
target.RefInput.Vel_bIc_des    = [wptsX_cell{1}(1,2) wptsY_cell{1}(1,2) wptsZ_cell{1}(1,2)];
target.RefInput.pos_des        = [wptsX_cell{1}(1,1) wptsY_cell{1}(1,1) wptsZ_cell{1}(1,1)];
target.RefInput.chi_des        = atan2(wptsY_cell{1}(1,2),wptsX_cell{1}(1,2));
target.RefInput.chi_dot_des    = 0;
target.RefInput.trajectory.refTime = [0 time_wptsX_cell{1}(end)];

simSetup
open(model)