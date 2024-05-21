function [SimOutAbb] = mySimOutFunc(SimOut)
% function [SimOutAbb] = mySimOutFunc(SimOut)
% 
% User Defined (abbreviated) SimOut function:  Used to downselect elements 
% of the full SimOut structure and assigns them to the new abbreviated bus
% SimOutAbb.  Note the some SimOut signals are "required" to be mapped from
% SimOut to SimOutAbb as the signals are required at the top level of GUAM
% NOTE: MAPPINGS ARE ARBITRARY (no requirement to maintain sub-bus
% structures, and math operations are permitted).
%
% e.g.:
% SimOutAbb.alpha = SimOut.Vehicle.EOM.AirRelativeData.alpha; % Permitted mapping
% SimOutAbb.alphanorm = sqrt(SimOut.Vehicle.EOM.AirRelativeData.alpha^2); % Permitted math mapping
%
% INPUT:
% SimOut: structure from GUAM bus which contains all major mapped bus
%   signals in GUAM
% OUTPUT
% SimOutAbb: abbreviated user selected output signals.

% Written by:
% Michael J. Acheson, michael.j.acheson@nasa.gov
% Dynamic Systems and Control Branch (D-316), NASA Langley Research Center (LaRC)
% 
% Versions:
% 9.5.2023 MJA: Initial Version completed


% *************************************************************************
% **** Make User Preferred SimOutAbb Structure Assignments Below **********
% *************************************************************************
SimOutAbb.Time2               = SimOut.Time;
SimOutAbb.Cmd2                = SimOut.Control.Cmd; % Some comment here...
SimOutAbb.Control.Cmd2        = SimOut.Control.Cmd;
SimOutAbb.NewTime             = SimOut.Time/5; % Some comment here...

% *************************************************************************
% ********* Fields required by top level of GUAM sim (below)  *************
% *************************************************************************
SimOutAbb.Vehicle.EOM.AirRelativeData.Veas          = SimOut.Vehicle.EOM.AirRelativeData.Veas;
SimOutAbb.Vehicle.EOM.AirRelativeData.Vtot          = SimOut.Vehicle.EOM.AirRelativeData.Vtot;
SimOutAbb.Vehicle.EOM.AirRelativeData.alpha         = SimOut.Vehicle.EOM.AirRelativeData.alpha;
SimOutAbb.Vehicle.EOM.AirRelativeData.beta          = SimOut.Vehicle.EOM.AirRelativeData.beta;
SimOutAbb.Vehicle.EOM.AirRelativeData.Vtotdot       = SimOut.Vehicle.EOM.AirRelativeData.Vtotdot;
SimOutAbb.Vehicle.EOM.AirRelativeData.alphadot      = SimOut.Vehicle.EOM.AirRelativeData.alphadot;
SimOutAbb.Vehicle.EOM.AirRelativeData.betadot       = SimOut.Vehicle.EOM.AirRelativeData.betadot;

SimOutAbb.Vehicle.EOM.WorldRelativeData.gamma       = SimOut.Vehicle.EOM.WorldRelativeData.gamma;
SimOutAbb.Vehicle.EOM.WorldRelativeData.chi         = SimOut.Vehicle.EOM.WorldRelativeData.chi;
SimOutAbb.Vehicle.EOM.WorldRelativeData.gammadot    = SimOut.Vehicle.EOM.WorldRelativeData.gammadot;
SimOutAbb.Vehicle.EOM.WorldRelativeData.chidot      = SimOut.Vehicle.EOM.WorldRelativeData.chidot;
SimOutAbb.Vehicle.EOM.WorldRelativeData.VelDtH_bEh  = SimOut.Vehicle.EOM.WorldRelativeData.VelDtH_bEh;
SimOutAbb.Vehicle.EOM.WorldRelativeData.Euler.phi   = SimOut.Vehicle.EOM.WorldRelativeData.Euler.phi;
SimOutAbb.Vehicle.EOM.WorldRelativeData.Euler.theta = SimOut.Vehicle.EOM.WorldRelativeData.Euler.theta;
SimOutAbb.Vehicle.EOM.WorldRelativeData.Euler.psi   = SimOut.Vehicle.EOM.WorldRelativeData.Euler.psi;

SimOutAbb.Vehicle.EOM.InertialData.Asensed_bIb      = SimOut.Vehicle.EOM.InertialData.Asensed_bIb;
SimOutAbb.Vehicle.EOM.InertialData.Pos_bii          = SimOut.Vehicle.EOM.InertialData.Pos_bii;
% *************************************************************************