% GUAM_Trim_Tests: The intent of this file is to utilize the current GAUM trim 
% file and its' associated controller to validate that the vehicle is well
% trimmed at both the (gain) scheduled trim points and at intermediate
% (mean) points between each schedule UH trim point for a specified WH
% value
%
% Input: 
% Name: Specify details of this input variable
% 
% Output:
% Name: Specify details of this output variable 
%
% Written by:
% Mike Acheson, Dynamic Systems and Control Branch (DSCB) D-316
% NASA Langley Research Center (LaRC), 7.26.2023
% 
% Modifications: Date/Initials/Details
% 
% 7.26.2023: MJA- Initial version of the m-script
% *************************************************************************
close all;
simSetup; % Need to run this to get access to SimIn quantities

% Define conversion variables
ft2kts = 1/(SimIn.Units.nmile/SimIn.Units.ft/3600); % Obtained from setUnits, Conversion factor of ft/sec to knots
kts2ft = (SimIn.Units.nmile/SimIn.Units.ft/3600); % Obtained from setUnits,  Conversion factor from knots to ft/sec
Sim_Name = 'GUAM'; % Simulink model name

% Specify Figure save details
save_flag = 1; % Flag to specify to export the trim plots
% fig_path = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver1p0'; % Trim_ver1p0
% fig_path = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver2p0'; % Trim_ver2p0
%fig_path = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver3p0_Update_U0'; % Trim_ver3p0
fig_path = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver4p0'; % Trim_ver4p0

% Setup the sim initially
% Define the userStruct inputs used for these test cases..
userStruct.variants.fmType= 2; % Specify use of Polynomial AP database
userStruct.variants.refInputType = 4; % Specify default use of doublets
userStruct.variants.eomType = 2; % 1 = STARS, 2= Simple
simSetup

% Specify the fixed WH
% ************ NEED TO SEE HOW TO IMPLEMENT THIS (I.E. TRIM VEHICLE WITH
% DESCENT RATE FROM TIME >= 0 SEC *****************************************
WH_des = 0/60; % ft/sec

% Create the UH test intervals (schedule and intermediate points)
UH_des_raw = []; UH_inter= []; UH_des=[];
UH_des_raw  = SimIn.Control.trim.UH;
UH_inter    = diff(UH_des_raw)/2+UH_des_raw(1:end-1);
UH_des      = sort([UH_des_raw; UH_inter]);
U_len       = length(UH_des);

fig_han = [];
% Execute the simulation for each test point and plot the trim results...
for loop = 1:U_len; % fails at 55, 58, 59
    % Set up the simulation 
    target = struct('tas', UH_des(loop)*ft2kts, 'gndtrack', 0, 'stopTime', 20);
    %target = struct('tas', 95, 'gndtrack', 0, 'stopTime', 20);

    simSetup; 
    SimPar.Value.RefInputs.doublets.Vel_finalValue(3) = 0; % Turn off the default doublet
    SimIn.Switches.FeedbackCurrent = 0; % Use the initial condition for all velocities
    %SimIn.Switches.FeedbackCurrent = 1; % Use the velocity CMD (Likely better choice)

    sim(Sim_Name); % Execute the simulation

    % Compute the actual velocity in command frame
    s = cos(SimOut.RefInputs.chi_des.Data/2);
    v = sin(SimOut.RefInputs.chi_des.Data/2);
    Q_i2c = [s, 0*v, 0*v, v]';
    Vel_bIc = Qtrans(Q_i2c,squeeze(SimOut.Vehicle.Sensor.Vel_bIi.Data));
    % Now plot the results

    fig_han{loop} = figure(loop);
    % x-axis
    subplot(3,1,1)
    plot(SimOut.Time.Data, SimOut.RefInputs.Vel_bIc_des.Data(:,1),'g');
    hold all
    plot(SimOut.Time.Data, Vel_bIc(1,:),'r');
    xlabel('Time (s)');
    ylabel('Vel (ft/sec)');
    legend('U_cmd', 'U_act','interpreter','none');
    title(sprintf('Trim Plot: UH = %6.2g, WH = %6.2g', UH_des(loop)*ft2kts, WH_des));
    %  Y-axis
    subplot(3,1,2)
    plot(SimOut.Time.Data, SimOut.RefInputs.Vel_bIc_des.Data(:,2),'g');
    hold all
    plot(SimOut.Time.Data, Vel_bIc(2,:),'r');
    xlabel('Time (s)');
    ylabel('Vel (ft/sec)');
    legend('V_cmd', 'V_act','interpreter','none');
    % z-axis
    subplot(3,1,3)
    plot(SimOut.Time.Data, SimOut.RefInputs.Vel_bIc_des.Data(:,3),'g');
    hold all
    plot(SimOut.Time.Data, Vel_bIc(3,:),'r');
    xlabel('Time (s)');
    ylabel('Vel (ft/sec)');
    legend('W_cmd', 'W_act','interpreter','none');
    fig_han{loop}.Name = sprintf('Trim_Plot_UH%0.4g_WH%0.3g',UH_des(loop)*ft2kts, WH_des);
    disp('');
    
    % If save_flag, then output the trim plots in the desired trim figure folder
    if save_flag
        Multi_Fig_Save_func(fig_path, 1.4, 1.4,'.fig','.png');
        close(fig_han{loop});
    end
    clear SimOut;
    
end





