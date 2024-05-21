% *************************************************************************  
% NASA TTT-Autonomous Systems(AS): Intelligent Contingency Management (ICM)  
%                       Generic UAM Simulation   
%                       Version 1.1  
% *************************************************************************  
%  
% *************************************************************************  
% Point of Contact:  
% Michael J. Acheson  
% NASA Langley Research Center (LaRC)  
% Dynamics Systems and Control Branch (D-316)  
% email: michael.j.acheson@nasa.gov  
% *************************************************************************  
%  
% Versions:  
% Version 1.1, 10.11.2023, MJA: Incorporates expanded polynomial   
% aero-propulsive database, trim tables and gain scheduled baseline controller.   
% Also inlcludes trim scripts (see ./vehicles/Lift+Cruise/Trim/trim_helix.x)   
% and linearization scripts (see ./vehcles/Lift+Cruise/Control/ctrl_scheduler_GUAM.m).  
% Also includes piece-wise Bezier RefInput reference trajectory capability, and a   
% simulation output animation script (./utilities/Animate_SimOut.m).  Also, this  
% version enables a user defined output script that enables user to specify output variables  
% (e.g., ./vehicles/Lift+Cruise/setup/User_SimOut/mySimOutFunc_Animate.m)  
%  
% *************************************************************************   
% To run a simulation example case, just execute the RUNME.m script at the top level!  
% **************************************************************************  
  
% This simulation is a generic UAM simulation.  It includes a generic   
% transition aircraft model representative of a NASA Lift+Cruise   
% vehicle configuration.    
%   
% Some of the key simulation components include:  
% 1) A simulation architecture (e.g., signal buses)  that supports the most   
% common rigid body 6-DOF frames of reference (e.g., Earth Centered Inertial,   
% Earth Centered Earth Fixed (ECEF), North-East-Down (NED), Navigation,   
% Velocity, Wind, Stability, and Body)  
% 2) A simulation architecture that contains most aerospace signals/quantities   
% of typical interest  
% 3) A generic architecture that readily supports swapping in and out aircraft   
% models, sensors, actuator models, control algorithms etc..  
% 4) A wide array of desired trajectory or RefInputs (e.g., ramps, timeseries,  
% piece-wise Bezier curves, and doublets)  
% 5) A nominal gain scheduled (LQRi) baseline,  
%  unified controller (same commands across three flight phases). The baseline   
% controller operates in the heading frame (i.e., the NED frame rotated by the   
% heading angle)  
%   
% Demonstration trajectory flights are found in the Exec_Scripts folder. Demo  
% cases include:    
%   1) a simple sinusoidal input case  (./Exec_Scripts/exam_TS_Sinusoidal_traj.m)  
%   2) a basic lifting hover and transition to forward flight (./Exec_Scripts/exam_TS_Hover2Cruise_traj.m)  
%   3) a cruise climbing right hand turn (./Exec_Scripts/exam_TS_Cruise_Climb_Turn_traj.m)  
%   4) a takeoff, climbing transition to cruise and descending deceleration to landing using ramps  
%   5) two examples of piece-wise Bezier curve trajectories: a) cruise decent and decel,   
%      b) hover climb and acceleration  
  
% These demonstration trajectories can be performed by executing ./RUNME.m at the top level folder   
% Alternatively, these examples can be accessed by adding the ./Exec_Scripts folder to the matlab path   
% running the associated execution example script (e.g., "exam_TS_Cruise_Climb_Turn_traj.m" or   
% "exam_TS_Hover2Cruise_traj.m" m-file).  Once the "GUAM" simulink model opens, run the model.    
% Output data is provided by the matlab logged signal logsout{1}.  Many analysis scripts make use  
% of the output data assigned to a SimOut structure: >>SimOut = logsout{1}.Values;  
% Simulation input (fixed) parameters are provided in a large structure SimIn, whereas   
% desired tunable simulation parameters are provided using the large structure: SimPar.    
% The structure SimIn, SimPar and SimOut therefore contain the (fixed) simulation inputs,   
% the (variable) simulation inputs, and the simulation outputs respectively.  Some basic  
% results plotting can be performed by running the m-file: ./vehicles/Lift+Cruise/Utils/simPlots_GUAM.m  
% Simulation results animation (e.g., creation of a .avi file or similar) is available by use  
% of the script: ./utilities/Animate_SimOut.m.    
% *************************************************************************  
  
% While the demonstration scripts are available, a user can in general select  
% among variants subsystem variants through an input structure "userStruct.variants"    
% then call the simSetup.m script to prepare the simulation for execution.  
% The simulation makes use of an input structure: "userStruct" to change the variants   
% of various subsystems (see ./setup/setupVariantStruct.m) that are used,   
% and to specify various "switches" (see ./setup/setupSwitches.m).  If a user  
% doesn't provide userStruct.variant selections, then the default choices   
% (vehicle specific) as set in ./vehicles/Lift+Cruise/setup/setupDefaultChoices.m.   
% An example of the available userStruct options are:  
%   userStruct.variants:  
%       refInputType: Timeseries  
%       vehicleType: LiftPlusCruise  
%       expType: DEFAULT  
%       atmosType: US_STD_ATMOS_76  
%       turbType: None  
%       ctrlType: BASELINE  
%       actType: FirstOrder  
%       propType: None  
%       fmType: Polynomial  
%       eomType: Simple  
%       sensorType: None  
%  
% The available options for the UserStruct.variants subfields are enumerations  
% specified in the "ClassDef" folder. For example the actuator type variants  
% available are found in the ActuatorEnum.m file:  
%  
% classdef ActuatorEnum < Simulink.IntEnumType  
%  enumeration  
%    None(1)  
%    FirstOrder(2)  
%    SecondOrder(3)   
%    FirstOrderFailSurf(4)   
%  end  
% end  
% A user can either write an m-file script or simply call (in the matlab command   
% line):  
% >>userStruct.variants.actType = 3; simSetup  
% Now the GUAM sim is ready to execute using the SecondOrder actuator model.  
%   
% Users have a few options to provide desired trajectories and basic flight   
% manuevers to execute.  The userStruct.variant.refInputType option selects    
% (look in the Lift+Cruise Reference Inputs variant subsytem block at the simulation  
% top level) between: FOUR_RAMP, ONE_RAMP, TIMESERIES, BEZIER, and DEFAULT (doublets) options.    
% The timeseries is demonstrated in the Exec_Scripts folder for the m-files   
% that contain "TS" in the name and they end with the suffix "_traj.m".   
%   
% A sample use case of the FOUR_RAMP is also contained in the Exec_Scripts, where   
% Simulink ramp blocks are used to build a simple trajectory.  In this option,   
% the user makes use of an input structure "target" to provide some basic trim  
% configuration information and then provides timing and magnitude for the   
% ramps blocks using the SimPar structure. Target field available for user   
% specification include: 'alt', 'tas', 'gndtrack', 'RefInput', 'Rate', and  
% 'stopTime'. A sample script "exam_RAMP.m" file demonstrates the use of both  
% the user provided target structure, methods to programmatically specify Ramp settings  
% (using SimPar), and the RAMP refInput variant subsytem.   
%  
% Generating and executing Piece-wise Bezier curve desired trajectory is shown  
% in the example: ./Exec_Scripts/exam_Bezier.m  This script makes use of two options:  
% 1) a user provided PW Bezier trajectory file (e.g., ./Exec_Scripts/exam_PW_Bezier_Traj.mat)  
% that must contain the structure: >> pwcurve.waypoints = {wptsX, wptsY, wptsZ};  
% and >> pwcurve.time_wpts = {time_wptsX, time_wptsY, time_wptsZ};  
% 2) or a "target" structure input the contains the PW Bezier information,  
% >> target.RefInput.Bezier.waypoints = {wptsX, wptsY, wptsZ};  
% >> target.RefInput.Bezier.time_wpts = {time_wptsX time_wptsY time_wptsZ};  
% along with the associated IC information (see ./Exec_Scripts/exam_Bezier.m for details)  
% Additionally, a PW Bezier curve plotting script is available ./Bez_Functions/Plot_PW_Bezier.m  
% for users to see the 3D trajectory, positions, velocities and accelerations (for each axis)   
% of the desired PW Bezier trajectory.   
%  
% Two aero-propulsive model options are available: a low-fidelity, first-principles-based   
% strip-theory model and a mid-fidelity, computationally-derived polynomial model. The aero-  
% propulsive model is selected using the "SFunction" and "Polynomial" fmType variant options,   
% respectively. The "SFunction" variant contains a matlab class/object based model that allows     
% a user to build an aircraft configuration (airfoil, number and location of rotors,  
% aerodynamic surfaces, etc.). The current aircraft configuration is a generic Lift+Cruise   
% configuration (for more details, see the contents of the  
% /vehicles/Lift+Cruise/AeroPro/SFuntion folder).  The "Polynomial" variant option is composed   
% of blended response surface equations identified from primarily computational fluid dynamics   
% (CFD) experiments for a generic Lift+Cruise configuration. The "SFunction" variant produces   
% aero-propulsive model predictions throughout the flight envelope, whereas the "Polynomial"   
% variant model limits may be exceeded and result in errors in some corners of the flight     
% envelope when users ask for model predictions in areas that are not characterized by the   
% response surface equations (the errors that may be encountered are intentional and intended   
% to prevent extrapolation outside of the region of validity of the polynomial model). The   
% polynomial aero-propulsive model executes much faster than the strip-theory "SFunction"   
% variant and is, therefore, the default option  
% (i.e., userStruct.variants.fmType = ForceMomentEnum.Polynomial)  
% References for the "SFunction" and "Polynomial" fmType variant options, respectively, are:  
% [1] Cook, J. W., and Hauser, J., "A Strip Theory Approach to Dynamic Modeling of  
%     eVTOL Aircraft," AIAA SciTech 2021 Forum, AIAA Paper 2021-1720, Jan. 2021.   
%     https://doi.org/10.2514/6.2021-1720.  
% [2] Simmons, B. M., Buning, P. G., and Murphy, P. C., “Full-Envelope Aero-Propulsive  
%     Model Identification for Lift+Cruise Aircraft Using Computational Experiments,”  
%     AIAA AVIATION 2021 Forum, AIAA Paper 2021-3170, Aug. 2021.   
%     https://doi.org/10.2514/6.2021-3170.  
%  
%  
% Basic simulation execution performance can be viewed via Simulink scopes   
% in the VehicleSimulation/Vehicle Generalized Control/Lift+Cruise/Control/Baseline   
% block.  
%  
% Simulation Trimming:  The (offline) trim routines are found in the ./vehicles/Lift+Cruise/Trim folder.  
% The top level trim routine is: trim_helix.m. This script was used to trim the overactuated Lift+Cruise     
% vehicle using the polynomial aero-propulsive database.  NOTE: the routine could also be used for trimming   
% with the strip theory s-function aero-propulsive model, but the code has not been modified to switch between   
% the models (likely not functional using the s-function model).  In the top level trim_helix.m script the user   
% specifies a range of forward and vertical velocities (could also provide a turn radius).  Next the user provides   
% some quantities needed for the quadratic cost function/optimization (e.g., initial guess, offset, scaling and free variables).  
% The quadratic cost function used by fmincon is: mycost.m, and the non-linear constraints function is nlinCon_helix.m.   
% The results of the trim table schedule is then saved in a .mat file.  
%   
% Baseline controller gain scheduling:  The gain scheduling m-files are contained in the ./vehicles/Lift+Cruise/control/ folder.  
% The top level script for gain scheduling the baseline controller (LSQi) is ctrl_scheduler_GUAM.m.  This script schedules the   
% Longitudinal and Lateral axes seperately.  A few linearization scripts are available but the main script is get_lin_dynamics_heading.m.  
% This script linearizes around a designated flight condition, and other scripts (e.g., get_lat_dynamics_heading.m and ctrl_lat.m) segregate  
% the linearized dynamics according to desired axes.  
%  
% *************************************************************************  
% Notices:  
% Copyright 2024 United States Government as represented by the Administrator   
% of the National Aeronautics and Space Administration. All Rights Reserved.  
% This software calls, but does not include, the following third-party software,   
% which is subject to the terms and conditions of its licensor, as applicable:  
%  
% The MATLAB®/SIMULINK® brand simulation software products,   
% which are offered by The MathWorks, Inc.  
%  
% Users must supply their own licenses:     https://www.mathworks.com  
%  
% Disclaimers  
% No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY  
% OF ANY KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT   
% LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO   
% SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A   
% PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT THE   
% SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION,   
% IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT,   
% IN ANY MANNER, CONSTITUTE AN ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR   
% RECIPIENT OF ANY RESULTS, RESULTING DESIGNS, HARDWARE, SOFTWARE PRODUCTS   
% OR ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT SOFTWARE.    
% FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING   
% THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE,   
% AND DISTRIBUTES IT "AS IS."   
%   
% Waiver and Indemnity:  RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST  
% THE UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL  
% AS ANY PRIOR RECIPIENT.  IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS  
% IN ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH  
% USE, INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM,   
% RECIPIENT'S USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND   
% HOLD HARMLESS THE UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS,  
% AS WELL AS ANY PRIOR RECIPIENT, TO THE EXTENT PERMITTED BY LAW.    
% RECIPIENT'S SOLE REMEDY FOR ANY SUCH MATTER SHALL BE THE IMMEDIATE,   
% UNILATERAL TERMINATION OF THIS AGREEMENT.  
