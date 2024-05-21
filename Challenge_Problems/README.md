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
% Challenge Problems: are broad open-research challenges that    
% Urban Air Mobility (UAM) and Advance Air Mobility (AAM) autonomy researchers  
% must overcome to enable safe, certified, autonomous operation of transition  
% vehicles. This repository provides a simulation, m-code & data that   
% enables formulation of increasingly difficult research challenges for the   
% autonomous UAM community  
%  
% Examples: collision avoidance, effector failure (control and flight   
% envelope estimation), traffic pattern entry, trajectory replanning, etc.  
%   
% How to use this repository:  
% Execute the RUNME.m script in the Challenge_Problems folder to see a demo  
% of an own-ship trajectory and failure scenario run in the GUAM simulation  
%   
% Contained within the Challenge_Problem folder are 4 m-files that were used  
% to create 4 separate data sets.  The intent of the data sets is to provide  
% a standardized set of trajectories, obstacles etc. to faciliate collaboration  
% and algorithm performance comparison across research groups.  The m-scripts  
% which where used to create the data are intended to provide both insight into   
% the data generation and to enable users to modify them for their specific   
% research needs. There are four data files that contain: own-ship trajectories,   
% stationary obstacles, moving obstacles and effector failure scenarios   
% respectively.  The data was constructed such that each "run" is correlated   
% across each data set.  For example, using own-ship trajectory 1 should  
% be used in conjunction with stationary obstacle 1, moving obstacle 1   
% and failure scenario 1.  However, other cases across the data sets may   
% still be used.  For example, using own-ship trajectory 1, there are many   
% other stationary obstacles that may be close enough to the trajectory to   
% require collision avoidance.  The m-file generation scripts include:  
%  
% Generate_Own_Traj.m: This script created Data_Set_1.mat, which contains   
%   2 separate sets of 3000 randomly created own-ship trajectories.  The first   
%   set of runs called own_traj_orig consist of simple Bernstein polynomial waypoints  
%   (think flight plan).  The second set of runs called own_traj consists of  
%   modification to the original flight plan (early turns, dynamically feasible    
%   turn entry etc.).  Users should utilize the own_traj Bernstein polynomial   
%   waypoints.  However, the own_traj_orig is available for users who wish to  
%   perform their own generation of dynamically feasible trajectories from   
%   a flight plan.   
%  
% Generate_Stat_Obst.m: This script was used to generate random stationary obstacles.  
%   Users can specify many parameters in the creation of obstacles (e.g., radius),  
%   however Data_Set_2.mat contains 3000 "runs" of obstacles that when used   
%   with the corresponding own-ship trajectory are guaranteed that some part of   
%   the obstacle is within 200 feet (see parameter max_perp_dist) of the trajectory   
%   at the prescribed trajectory time. No collision avoidance algorithm is  
%   released in GUAM v1.1 and users are required to provide their own avoidance  
%   methodology.  
%   
% Generate_Mov_Obst.m: This script is used to generate moving obstacles. Similar  
%   to the stationary obstacles, the moving obstacles are randomly created   
%   such that they are within 200 feet (see parameter max_perp_dist) of the   
%   corresponding own-ship trajectory at the randomly chosen "collision" time.  
%   Additionally, a short, randomly created rectilinear Bernstein polynomial   
%   trajectory is included.  No collision avoidance algorithm is  
%   released in GUAM v1.1 and users are required to provide their own avoidance  
%   methodology.  
%     
% Generate_Failures.m: This script is used to randomly generate aerodynamic   
%   control surface and propulsor failures at a random time during the   
%   corresponding own-ship trajectory.  These failures are limited to hold-last,  
%   limited pre-scale (effector command limiting prior to effector dynamics), post  
%   scale(effector output limiting after effector dynamics) and effector reversal   
%   applicable to aerodynamic effectors only.  Other effector failure types are  
%   available but not demonstrated here.  
%  
% Plotting Utility: a data set plotting utility is provided (Plot_Chal_Prob_Sets.m)   
%   This script can plot one or more own-ship trajectories, stationary obstacles,  
%   and the moving obstacle & their trajectory  
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