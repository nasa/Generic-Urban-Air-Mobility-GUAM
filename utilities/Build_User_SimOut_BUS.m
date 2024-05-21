function [] = Build_User_SimOut_BUS(userFname, Prefix, topBus)
% *************************************************************************
% This script is used to create a user defined bus from the down-selected
% elements of SimOut. The bus that is created is assigned to the GUAM
% output block to facilitate execution of GUAM using the abbreviated output
% 
% INPUTS:
% userFname: string which is the name of the user provided (or default)
%   mapping .m script (e.g., ./vehicles/Lift+Cruise/setup/defaultSimOut.m
%   that (down) selects desired elements of SimOut and outputs a reduced 
%   structure (e.g. SimOutAbb)
% Prefix: string that contains a the desired prefix used in resulting bus
%   objects in the main workspace e.g., 'BUS_USER_'
% topBus: top level bus object (e.g., BUS_SIM_OUT) 
%
% OUTPUTS:
% None:

% Created by Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), Dynamic Systems and Control Branch (D-316)
% 
% Modifications:
% 9.7.2023, MJA: Initial version
% *************************************************************************

% *************************************************************************
% Define a function handle for userFname
f_hand = str2func(userFname);
% Create a temporary (existing SimOut)
SimOut_temp = Simulink.Bus.createMATLABStruct(topBus);
% Pass the temporary Structure to f_hand to reshape the desired output bus
SimOutAbb = f_hand(SimOut_temp);

% Now build the output buses...
[b] = buildBusObject(SimOutAbb, Prefix,'SIM_OUT');
assignin('base',strcat(Prefix, 'SIM_OUT'), b);
    
% *************** Update two GUAM model blocks ****************************
full_bus_name = sprintf('%s%s',Prefix,'SIM_OUT');
load_system('GUAM')
% Now set the new output bus to the output simulink block
model_name1 = 'GUAM/Vehicle Simulation/SimOut_Selection';
bd = get_param(gcs,"Object");
block = find(bd,"-isa","Stateflow.EMChart", ...
    Path=model_name1);
block.Outputs.DataType = sprintf('Bus: %s',full_bus_name);

modelBlock_name2 = 'GUAM/Vehicle Simulation/SimOutputs';
set_param(modelBlock_name2,'OutDataTypeStr',sprintf('Bus: %s',full_bus_name));
save_system('GUAM','SaveDirtyReferencedModels','on');
% *************************************************************************
end