%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType = 4; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)

%clear pwcurve;
%% Define the target structure and provide the Ramp settings
% target = struct('tas', 0, 'gndtrack', 0,'stopTime', 30);
PW_Bezier_flag = 0; % Flag to denote failure of PW Bezier setup
% *************************************************************************
% Picks the piece-wise Bezier (Bernstein Polynomial) curve (two methods)
pick_flag = 0;
while ~pick_flag
    u_choice = input(sprintf('Select: 1 or 2:\n(1) Use target structure\n(2) Use userStruct.trajFile (trajectory file)\nUser Input: '));
    switch u_choice
        case 1 % Use of target structure
            % First create the example trajectory (simple liftoff in hover and transition to fwd flight):
            % pos in ft, vel in ft/sec, and acc in ft/sec^2
            wptsX = [0 100 0; 2000 100 0; 3500 50 0]; % within row = pos vel acc, rows are waypoints
            time_wptsX = [0 20 40];
            wptsY = [0 0 0; 0 0 0]; % within row = pos vel acc, rows are waypoints
            time_wptsY = [0 40];
            wptsZ = [0 0 0; 0 0 0; 83.33 500/60 0]; % within row = pos vel acc, rows are waypoints, NOTE: NED frame -z is up...
            time_wptsZ = [0 20 40];
            % NOTE each axis is handled seperately and can have different number of rows (waypoints and times), 
            % however start and stop times must be consistent across all three axes

            % Store the PW Bezier trajectory in the target structure (used in RefInputs)
            target.RefInput.Bezier.waypoints = {wptsX, wptsY, wptsZ};
            target.RefInput.Bezier.time_wpts = {time_wptsX time_wptsY time_wptsZ};

            % Set desired Initial condition vars
            target.RefInput.Vel_bIc_des    = [100;0;0];
            target.RefInput.pos_des        = zeros(3,1);
            target.RefInput.chi_des        = 0;
            target.RefInput.chi_dot_des    = 0;
            target.RefInput.trajectory.refTime = [0 40];

            % Plot the sample PW Bezier curve that was created (see visualization of trajectory and derivatives)
            Plot_PW_Bezier;
            clear wptsX wptsY wptsZ time_wptsX time_wptsY time_wptsZ 
            userStruct.trajFile = ''; % Delete user specified PW Bezier file

            pick_flag = 1; % Exit while loop as user selected Bezier method
        case 2 % Use of designated Bezier trajectory file..
            % The structure of fields necessary in trajectory file include:
            % pwcurve.waypoints = {wptsX, wptsY, wptsZ};
            % pwcurve.time_wpts = {time_wptsX, time_wptsY, time_wptsZ};
    
            % First create the example trajectory (simple liftoff in hover and transition to fwd flight):
            % pos in ft, vel in ft/sec, and acc in ft/sec^2
            wptsX = [0 0 0; 0 0 0; 1750 50 0]; % within row = pos vel acc, rows are waypoints
            time_wptsX = [0 10 80];
            wptsY = [0 0 0; 0 0 0]; % within row = pos vel acc, rows are waypoints
            time_wptsY = [0 80];
            wptsZ = [0 0 0; -80 -500/60 0;-580 -500/60 0]; % within row = pos vel acc, rows are waypoints, NOTE: NED frame -z is up...
            time_wptsZ = [0 20 80];
            % NOTE each axis is handled seperately and can have different number of rows (waypoints and times), 
            % however start and stop times must be consistent across all three axes
    
            % Create the structure and save into the trajectory file
            pwcurve.waypoints = {wptsX, wptsY, wptsZ};
            pwcurve.time_wpts = {time_wptsX, time_wptsY, time_wptsZ};
            save('./Exec_Scripts/exam_PW_Bezier_Traj','pwcurve','-v7.3');
            if exist('target','var')
                clear target;
            end
            
            % Specify the file just created in userStruct for simSetup.m to load
            userStruct.trajFile = './Exec_Scripts/exam_PW_Bezier_Traj.mat';
    
            % Plot the sample PW Bezier curve that was created (see visualization of trajectory and derivatives)
            Plot_PW_Bezier;
            clear pwcurve wptsX wptsY wptsZ time_wptsX time_wptsY time_wptsZ 

            pick_flag = 1; % Exit while loop as user selected Bezier method
        otherwise
            fprintf('User needs to supply selection choice (1-2)\n')
    end
end
% *************************************************************************

% Initialize the sim
simSetup;

% Open the simulation
open(model);