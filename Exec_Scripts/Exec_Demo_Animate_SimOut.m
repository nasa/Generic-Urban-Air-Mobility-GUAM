% Animate_SimOut.m: This script is used to animate GUAM simulation results.  
% It utilizes the simulation output variable SimOut (generated from the 
% logsout{1} variable) to provide necessary state information 
% (e.g., positions, effector positions, etc..) to animate the simulation 
% results using the strip theory aero propulsive s-function draw methods of
% tiltwing class.
% NOTE: the simulation output could be produced by either the strip theory
% s-function or the polynomial aero-propulsive model, but the strip theory
% tilt wing class object is ONLY used in this script to animate the output.

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 9.13.2023, MJA: Initial version of script.  Makes use of tiltwing object
% class method 'draw' created by Jacob Cook, and portions of animation
% scripts originally written by Matthew Houghton and Alex Oshin

% *********************** User Input **************************************
close all;
%DataFname       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\Exec_Scripts\Hover2CruiseSimOut.mat'; % Input SimOut filename or ''
DataFname       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\Exec_Scripts\Cruise_Climb_Turn_traj.mat'; % Input SimOut filename or ''
VideoOutFname   = './Cruise_Climb_Turn_traj.avi'; % Desired filename of output video

% User String to put in movie title
TitStr          ='Demo Video of RUNME: Cruise Climb Turn,'; % User String to put in movie title

% Specify desired start/stop simulation times to display in .avi movie
t_start         = 0; % Specify simulation time to start .avi movie
t_end           = []; % Specify time (after start time) or [] for last time is file
% Following variables are used to scale the vehicle to ensure it is visible in .avi movie
base_acSize     = 60; % Rough size of plotted (stationary) a/c in ft
des_scale       = 0.20; % Desired scale size of aircraft in plot (factor used to keep aircraft size visible in long trajectories)

% Set surface annotation location [x y w h]
annot_loc = [0.05 0.01 0.3 0.3];

% Video frame rate (e.g., 50 frames/sec), should be integer multiple of simulation rate
FrameRate       = 30; % Desired frames/sec in video
FR_factor       = 5; % Frame rate scale factor (speed up video), e.g., FR_factor=2; => one second of realtime in video corresponds to 2 secs in simulation time

% set Matlab view command az and elev offsets
az_offset       = -45; % Offset angle added to matlab "view(az,el)" azimuth command
el_offset       = 20; % Offset angle added to matlab "view(az,el)" elevation command

% *************************************************************************

% ********* Load or create required variables, objects etc ****************
% Obtain Simulation SimOut Data
if ~isempty(DataFname)
    % Obtain SimOut from previously saved simulation output
    load(DataFname); % Load in SimOut data from a previously saved run
else
    % Create SimOut variable (after simulation execution) from logsout{1} logged data
    SimOut = logsout{1}; % Assign the logged simulation output to SimOut
end

% Check if tiltwing class object exists, otherwise create it (reqd for draw methods)
if ~exist('tiltwing','var')
    tiltwing = build_Lift_plus_Cruise;
end
% *************************************************************************

% ********************* Animate Simulation Output
% *************************
clear tObj TraceHand DesTrajHand a_hand
tStart_ind = find(SimOut.Time.Data>=t_start,1,'first');
if isempty(t_end)
    tEnd_ind = length(SimOut.Time.Data);
else
    tEnd_ind   = find(SimOut.Time.Data<=t_end,1,'last');
end
vid = VideoWriter(VideoOutFname);
open(vid);

ax = axes;
xlabel('X (ft)')
ylabel('Y (ft)')
zlabel('Z (ft)')
hold all

% Determine time indices step interval based on desired FrameRate and 
% Frame Rate factor..
del_t = (SimOut.Time.Data(2)-SimOut.Time.Data(1));
if 1/FrameRate < del_t
    step_int = 1;
else 
    step_int = round(1/FrameRate/del_t);
end

first_pass_flag = 0; % Flag used to indicate after first pass thru animation
for ind = tStart_ind:step_int*FR_factor:tEnd_ind

    tiltwing.om_p = [ SimOut.Vehicle.PropAct.EngSpeed.Data(ind,:) ];
    tiltwing.del_f = (SimOut.Vehicle.SurfAct.Position.Data(ind,1) + SimOut.Vehicle.SurfAct.Position.Data(ind,2))/2;
    tiltwing.del_a = SimOut.Vehicle.SurfAct.Position.Data(ind,1)-tiltwing.del_f;
    
    tiltwing.del_e = SimOut.Vehicle.SurfAct.Position.Data(ind,3);
    tiltwing.del_r = SimOut.Vehicle.SurfAct.Position.Data(ind,5);
    %% Set up the state conditions
    rho = SimOut.Env.Atmosphere.Density.Data(ind);
    vb_bIi = SimOut.Vehicle.EOM.InertialData.Vel_bIi.Data(ind,:)';
    om = SimOut.Vehicle.EOM.InertialData.Omeg_BIb.Data(ind,:)';
    
    %% calculate the aerodynamic forces and moments
    % tiltwing.aero(rho,vb_bIi, om,0);
    tiltwing = tiltwing.aero(rho, vb_bIi, om, 1);

    % Delete text annotation if it exist
    if first_pass_flag
        delete(a_hand)
    end

    % Delete the last aircraft object
    if first_pass_flag
        delete(tObj);
        % plot the trace line
        if exist('TraceHand','var') 
            % Update trace for own aircraft (as flown traj)
            set(TraceHand,'XData', [TraceHand.XData SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,1)]);
            set(TraceHand,'YData', [TraceHand.YData SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,2)]);
            set(TraceHand,'ZData', [TraceHand.ZData SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,3)]);
            % Update trace for desired trajectory
            set(DesTrajHand,'XData', [DesTrajHand.XData SimOut.RefInputs.pos_des.Data(ind, 1)]);
            set(DesTrajHand,'YData', [DesTrajHand.YData SimOut.RefInputs.pos_des.Data(ind, 2)]);
            set(DesTrajHand,'ZData', [DesTrajHand.ZData SimOut.RefInputs.pos_des.Data(ind, 3)]);
        end
    else
        TraceHand = plot3(SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,1), SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,2),...
            SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,3),'g.'); % Trace created for own aircraft (as flown traj)
        DesTrajHand = plot3(SimOut.RefInputs.pos_des.Data(ind, 1), SimOut.RefInputs.pos_des.Data(ind, 2), SimOut.RefInputs.pos_des.Data(ind, 3),'k.'); % Trace for desired traj
    end

    % Create the aircraft object   
    h = tiltwing.draw(rho,vb_bIi);
    tObj = hgtransform('Parent',ax);
    set(h,'Parent',tObj);
    % Create object transforms..
    Tr_Obj = makehgtform('translate',[SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,1), SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,2),...
        SimOut.Vehicle.EOM.InertialData.Pos_bii.Data(ind,3)]);
    Rotx_Obj = makehgtform('xrotate',SimOut.Vehicle.EOM.WorldRelativeData.Euler.phi.Data(ind));
    Roty_Obj = makehgtform('yrotate',SimOut.Vehicle.EOM.WorldRelativeData.Euler.theta.Data(ind));
    Rotz_Obj = makehgtform('zrotate',SimOut.Vehicle.EOM.WorldRelativeData.Euler.psi.Data(ind));
    
    alf = atan2(SimOut.Vehicle.EOM.InertialData.Vel_bIi.Data(ind,3),SimOut.Vehicle.EOM.InertialData.Vel_bIi.Data(ind,1));
    Rot_Init = makehgtform('yrotate',-alf);
    cur_AxSize = axis;
    ax_dim = 0;
    for AxLoop = 1: length(cur_AxSize)/2
        ax_dim(AxLoop) =cur_AxSize(AxLoop*2)-cur_AxSize(AxLoop*2-1);
    end
    ax_maxLen = sqrt(ax_dim*ax_dim'); % Note size of L+C ~60
    if base_acSize/ax_maxLen > des_scale
        obj_scale = 1;
    else
        obj_scale = des_scale*ax_maxLen/base_acSize;
    end
    Scale_Obj = makehgtform('scale', obj_scale);
    % Now translate, scale and rotate the object
    set(tObj,'Matrix', Tr_Obj*Scale_Obj*Rotz_Obj*Roty_Obj*Rotx_Obj*Rot_Init);
    sur_str = sprintf('Flap: %4.2f (deg)\nAil: %4.2f (deg)\nElev: %4.2f (deg)\nRud: %4.2f (deg)',(SimOut.Vehicle.SurfAct.Position.Data(ind,1) + SimOut.Vehicle.SurfAct.Position.Data(ind,2))/2*180/pi, ...
        (SimOut.Vehicle.SurfAct.Position.Data(ind,1)-(SimOut.Vehicle.SurfAct.Position.Data(ind,1) + SimOut.Vehicle.SurfAct.Position.Data(ind,2))/2)*180/pi,...
        SimOut.Vehicle.SurfAct.Position.Data(ind,3)*180/pi, SimOut.Vehicle.SurfAct.Position.Data(ind,5)*180/pi);
    a_hand = annotation('textbox',annot_loc, 'String', sur_str, 'FitBoxToText','on','color','red');

    % Update the view based on aircraft heading (e.g., chase plane view)
    az_deg = -SimOut.Vehicle.EOM.WorldRelativeData.Euler.psi.Data(ind)*180/pi + az_offset;
    view(az_deg,el_offset)

    title(sprintf('%s Time:%5.2f', TitStr, SimOut.Time.Data(ind)));
    legend('As Flown Traj','Desired Traj','Location', 'southoutside');

    drawnow;
    frame = getframe(gcf);
    writeVideo(vid,frame);
    first_pass_flag = 1;
end
close(vid)
% *************************************************************************