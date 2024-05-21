function [wptsX_out, wptsY_out, time_wptsX_out, time_wptsY_out, Circ_cent, err_flag, xy_ind_out, num_added] = ...
    ComputePreTurn(grav, xy_index, wptsX, wptsY, time_wptsX, time_wptsY, ...
    turn_rate, max_phi, phi_time, min_turn_ang, pre_flag)

% This function is used to generate a "preturn" for precisely turning
% between two different STRAIGHT LINE trajectory segments. The turn is assumed 
% to be at constant x/y velocity (constant AOB turn).  The preturn is 
% turn required to precisely leave one segment at a specified turn rate/max
% phi and precisely join the second segment on the correct heading.  The
% input requires 3 consecutive piecewise Bernstein polynomial wpts 
% (consider x and y plane only) and their associated time vectors.  The 
% output adds in the wpts to make the required turn 
% (while also deleting the original interior waypoint). If the turn angle is
% less than the min_turn_ang provided then no pre-turn in generated.  
% Additionally, if the preturn requires before the first segment starts, or
% after the second segment ends than no turn is generated and an error flag
% is set and no wpt changes are made
%
% Inputs:
% grav:         Earths gravitational constant (units in ft/sec^2 or m/sec^2)
% xy_index:     2x1 vector of index in x & y waypoints that is the first
%               linear segment
% wptsX:        nx3 piecewise bezier x-waypoints (position, velocity, accel)
% wptsY:        nx3 piecewise bezier y-waypoints (position, velocity, accel)
% wptsZ:        nx3 piecewise bezier z-waypoints (position, velocity, accel)
% time_wptsX:   nx1 vector of times for each x-waypoint
% time_wptsY:   nx1 vector of times for each y-waypoint
% time_wptsZ:   nx1 vector of times for each z-waypoint
% turn_rate:    desired turn rate (rad/sec) when established in turn
% max_phi:      maximum allowed angle-of-bank
% min_turn_ang: smallest heading angle change for which a preturn is
%               computed
% pre_flag:     flag to do smooth pre-turn transition pre_flag~=0 =>
%               smooth, else => no smooth transition
%
% Outputs:
% wptsX_out:        modified list of x-piecewise Bernstein polynomials
% wptsY_out:        modified list of y-piecewise Bernstein polynomials
% time_wptsX_out:   modified list of times for each x-waypoint
% time_wptsY_out:   modified list of times for each y-waypoint
% circ_cent:        this is the computed center of rotation...
% err_flag:         error flag (non-zero is error)
% xy_ind_out:       Output 
% num_added         Number of pre-turn waypoints added

% Written by Mike Acheson, NASA Langley Research Center
% Dynamic Systems and Control Branch (D-316)
% michael.j.acheson@nasa.gov

% Modification History:
% 3/29/2024 MJA: original function written.

% *************************************************************************
err_flag    = 0; % Initialize the error flag
 
% Now compute the bank angle required to get the desired turn rate, if the
% bank exceeds the bank max, then limit the bank (and new max turn rate) to
% find the limited max turn radius
Vtotal      = sqrt(wptsX(xy_index(1),2)^2+wptsY(xy_index(2),2)^2);
phi_act     = atan(Vtotal*turn_rate/grav);
if phi_act <= max_phi
    % Not AOB limited
    R               = Vtotal/turn_rate;
    turn_rate_act   = turn_rate;
else
    % Limit AOB to max allowed
    R               = Vtotal^2/grav/tan(max_phi);
    turn_rate_act   = Vtotal/R;
    phi_act         = max_phi;
end

% Compute the linear segment vectors and their unit vectors  
L1      = [wptsX(xy_index(1)+1,1)-wptsX(xy_index(1),1); wptsY(xy_index(2)+1,1)-wptsY(xy_index(2),1)];
L1_hat  = L1./norm(L1); % Points from the start of the first segment to the common point (along the direction of travel)
L2      = [wptsX(xy_index(1)+2,1)-wptsX(xy_index(1)+1,1); wptsY(xy_index(2)+2,1)-wptsY(xy_index(2)+1,1)];
L2_hat  = L2./norm(L2); % Points from the common point to the next wypt (along the direction of travel)
% psi_start   = atan2(wptsY(xy_index(2),2),wptsX(xy_index(2),2));
psi_L1   = atan2(L1_hat(2),L1_hat(1));
psi_L2   = atan2(L2_hat(2),L2_hat(1));

% If the angle between the direction of travel is less than the min_angle
% then perform 2-sided roll maneuver (compute time reqd to perform
% maneuver)
lin_ang = acos(L1_hat'*L2_hat);
if lin_ang>0 && lin_ang<min_turn_ang || ...
    lin_ang<0 && lin_ang>-abs(min_turn_ang)

    % Compute the time required for 2-sided roll maneuver
    % psi_L1    = atan2(L1_hat(2),L1_hat(1)); % Angle of first linear segment
    % psi_L2    = atan2(L2_hat(2),L2_hat(1)); % Angle of second linear segment
    turn_sign = sign(psi_L2-psi_L1); % Positive is right turn

    % Compute the maneuver time required
    [phi_time, ~, psi_del_quart] = Find_Roll_2Side_Tf(grav, Vtotal, 0, turn_sign*phi_act, psi_L2-psi_L1);
    % Compute the maneuver distances...
    [Px, Py, psi_delta] = Roll_Trans_2Side(grav, Vtotal, 0, turn_sign*phi_act, phi_time, psi_L1, 1);

    % Solve for the locations on L1 and L2 to start and end the transitions
    k           = [L1_hat L2_hat]\[Px;Py];
    PtA         = [wptsX(xy_index(1)+1,1);wptsY(xy_index(2)+1,1)]-k(1)*L1_hat;
    PtB         = [wptsX(xy_index(1)+1,1);wptsY(xy_index(2)+1,1)]+k(2)*L2_hat;
    time_PtA    = norm(PtA-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/norm(L1)*(time_wptsX(xy_index(1)+1)-time_wptsX(xy_index(1)))+time_wptsX(xy_index(1));
    time_PtB    = time_PtA + phi_time;

    % Copy the existing waypoints up to and including the first point in
    % the first linear segment
    wptsX_out(1:xy_index(1),:)  = wptsX(1:xy_index(1),:);
    wptsY_out(1:xy_index(1),:)  = wptsY(1:xy_index(1),:);
    time_wptsX_out              = time_wptsX(1:xy_index(1)); 
    time_wptsY_out              = time_wptsY(1:xy_index(2));
    % Now insert the manuever points
    wptsX_out(xy_index(1)+1,:)    = [PtA(1) Vtotal*L1_hat(1) 0];
    wptsX_out(xy_index(1)+2,:)    = [PtB(1) Vtotal*L2_hat(1) 0];
    wptsY_out(xy_index(2)+1,:)    = [PtA(2) Vtotal*L1_hat(2) 0];
    wptsY_out(xy_index(2)+2,:)    = [PtB(2) Vtotal*L2_hat(2) 0];
    time_wptsX_out(xy_index(1)+1) = time_PtA;
    time_wptsX_out(xy_index(1)+2) = time_PtB;
    time_wptsY_out(xy_index(2)+1) = time_PtA;
    time_wptsY_out(xy_index(2)+2) = time_PtB;

    % Now insert the remaining waypoints
    wptsX_out                       = [wptsX_out; wptsX(xy_index(1)+2:end,:)];
    wptsY_out                       = [wptsY_out; wptsY(xy_index(2)+2:end,:)];
    time_at_Pt3                     = time_PtB + norm([wptsX(xy_index(1)+2,1);wptsY(xy_index(2)+2,1)]-PtB)/Vtotal;
    del_time                        = time_wptsX(xy_index(1)+2)-time_at_Pt3;
    time_wptsX_out(xy_index(1)+3)   = time_at_Pt3;
    time_wptsX_out                  = [time_wptsX_out time_wptsX(xy_index(1)+3:end)-del_time];
    time_wptsY_out(xy_index(2)+3)   = time_at_Pt3; 
    time_wptsY_out                  = [time_wptsY_out time_wptsY(xy_index(2)+3:end)-del_time];
    xy_ind_out = xy_index + 2;
    
    num_added = 2; % Added two waypoints
    err_flag = 0; % No error fault
    Circ_cent = []; % No circ to output
    disp('');
else
    % Compute a larger angle turn transition: one-sided turn, circular arc and one-sided turn
    % Compute the normal vectors
    turn_sign = sign(psi_L2-psi_L1); % Positive is right turn

    if pre_flag % Utilize a dynamically feasible turn entry
        % *********************************************************************
        % Now compute the entry roll transition information
        phi_start1   = 0;
        phi_end1     = turn_sign*phi_act; % Set the ending phi to the actual phi
        % Solve for the delta distances and delta heading during the roll transition for Line 1
        [Pxt, Pyt, psi_delta] = Roll_Trans_1Side(grav, Vtotal, phi_start1, phi_end1, phi_time, psi_L1);
        Pxy1        = [Pxt;Pyt];
        % Solve for the delta distances and delta heading during the roll transition for Line 2
        [Px2, Py2, psi_delta] = Roll_Trans_1Side(grav, Vtotal, phi_end1, 0, phi_time, psi_L2-psi_delta);
        Pxy2        = [Px2;Py2];

        ang_n1      = psi_L1 + turn_sign*pi/2+psi_delta; 
        ang_n2      = psi_L2 + turn_sign*pi/2-psi_delta;
        n1_hat      = [cos(ang_n1);sin(ang_n1)];
        n2_hat      = [cos(ang_n2);sin(ang_n2)];
        
        % Solve for k scaling
        k           = [L1_hat -L2_hat]\(R*(n2_hat-n1_hat)-Pxy1-Pxy2);
        E1a         = L1_hat*k(1)+[wptsX(xy_index(1)+1,1);wptsY(xy_index(2)+1,1)];
        E1a_time    = norm(E1a-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/norm(L1)*...
                      (time_wptsX(xy_index(1)+1)-time_wptsX(xy_index(1)))+time_wptsX(xy_index(1));
        I1a         = E1a+Pxy1;
        Circ_cent   = I1a + R*n1_hat;
        I1a_time    = E1a_time + phi_time;

        % Compute the exit Early Turn Point (point that rejoins L2)
        E2a        = L2_hat*k(2)+[wptsX(xy_index(1)+1,1);wptsY(xy_index(2)+1,1)];
    
        % Compute the circle intersection points at the beginning of the turn roll out 
        % and the times of the arc and straight line point time
        I2a        = E2a-Pxy2;
        I2a_time   = abs(ang_n1-ang_n2)*R/Vtotal+I1a_time;
        E2a_time   = I2a_time+phi_time;

        % *********************************************************************
        % Initialize the waypoints_out with the waypoints_in up to the first index waypoint
        wptsX_out(1:xy_index(1),:) = wptsX(1:xy_index(1),:); wptsY_out(1:xy_index(1),:) = wptsY(1:xy_index(1),:);
        time_wptsX_out = time_wptsX(1:xy_index(1)); 
        time_wptsY_out = time_wptsY(1:xy_index(2));
    
        % Add the Early Turn Point and the associated circle intersection point
        wptsX_out(xy_index(1)+1,:) = [E1a(1) L1_hat(1)*Vtotal 0];
        wptsX_out(xy_index(1)+2,:) = [I1a(1), cos(psi_L1+psi_delta)*Vtotal, n1_hat(1)*grav*tan(abs(phi_end1))];
        wptsY_out(xy_index(2)+1,:) = [E1a(2) L1_hat(2)*Vtotal 0];
        wptsY_out(xy_index(2)+2,:) = [I1a(2), sin(psi_L1+psi_delta)*Vtotal, n1_hat(2)*grav*tan(abs(phi_end1))];

        % Now add the second circle intersection point and the L2 intersection
        wptsX_out(xy_index(1)+3,:) = [I2a(1) cos(psi_L2 - psi_delta)*Vtotal n2_hat(1)*grav*tan(abs(phi_end1))];
        wptsX_out(xy_index(1)+4,:) = [E2a(1), L2_hat(1)*Vtotal, 0];
        wptsY_out(xy_index(2)+3,:) = [I2a(2) sin(psi_L2 - psi_delta)*Vtotal n2_hat(2)*grav*tan(abs(phi_end1))];
        wptsY_out(xy_index(2)+4,:) = [E2a(2), L2_hat(2)*Vtotal, 0];
    
        % Add the end waypoint of the line segment L2 and any remaining waypoints...
        wptsX_out  = [wptsX_out; wptsX(xy_index(1)+2:end,:)];
        wptsY_out  = [wptsY_out; wptsY(xy_index(2)+2:end,:)];
    
        % Now update the time vectors...
%         time_PtB + norm([wptsX(xy_index(1)+2,1);wptsY(xy_index(2)+2,1)]-PtB)/Vtotal;
        time_L2_E2a = E2a_time + norm([wptsX(xy_index(1)+2,1);wptsY(xy_index(2)+2,1)]-E2a)/Vtotal;
%         time_L2_E2a = norm([wptsX(xy_index(1)+2,1);wptsY(xy_index(2)+2,1)]-E2a)/norm(L2)*...
%             (time_wptsX(xy_index(1)+2)-time_wptsX(xy_index(1)+1))+E2a_time;
        time_wptsX_out = [time_wptsX_out E1a_time I1a_time I2a_time E2a_time time_L2_E2a time_wptsX(xy_index(1)+3:end)];
        time_wptsY_out = [time_wptsY_out E1a_time I1a_time I2a_time E2a_time time_L2_E2a time_wptsY(xy_index(2)+3:end)];
        
        % Update the xy_index
        xy_ind_out = xy_index + 4;

        num_added = 4; % Added two waypoints

    else % Don't use a roll transition (just hit the pts tangential to the final turn radius)
        
        % Solve for the scaling along the original segments to find circle
        % intersections along each linear segment
        if turn_sign > 0
            Norm_rot_ang = pi/2;
        else
            Norm_rot_ang = -pi/2;
        end
        N1_hat = [cos(Norm_rot_ang) -sin(Norm_rot_ang); sin(Norm_rot_ang) cos(Norm_rot_ang)]*L1_hat;
        N2_hat = [cos(Norm_rot_ang) -sin(Norm_rot_ang); sin(Norm_rot_ang) cos(Norm_rot_ang)]*L2_hat;
        K           = [L1_hat L2_hat]\(R*(+N1_hat-N2_hat)); % Least squares solution
        PtA         = [wptsX(xy_index(1)+1,1);wptsY(xy_index(1)+1,1)]-L1_hat*K(1);
        PtB         = [wptsX(xy_index(1)+1,1);wptsY(xy_index(1)+1,1)]+L2_hat*K(2);
        Circ_cent   = PtA + R*N1_hat;
        % Don't need Circ_cent2 as should be identical to Circ_cent (here just for troubleshooting)
        % Circ_cent2  = PtB + R*N2_hat; 
        % **************************************************************************
        % Check if pre-turn point occurs outside the current segments (before the first,
        % or after the second). If so then exit with an error...
        if (norm(L1) < norm(PtA-[wptsX(xy_index(1)+1,1); wptsY(xy_index(2)+1,1)])) || ...
           (norm(L2) < norm(PtB-[wptsX(xy_index(1)+1,1); wptsY(xy_index(2)+1,1)]))
            err_flag = 2;
            wptsX_out=wptsX; wptsY_out=wptsY; time_wptsX_out=time_wptsX; time_wptsY_out=time_wptsY;
            return; % Exit as preturn is before the first segment begins or after the last segment ends...
        end
        % *************************************************************************
        
        % Update the X&Y wpts and time outputs
        wptsX_out(1:xy_index(1),:) = wptsX(1:xy_index(1),:); wptsY_out(1:xy_index(1),:) = wptsY(1:xy_index(1),:);
        time_wptsX_out = time_wptsX(1:xy_index(1)); time_wptsY_out = time_wptsY(1:xy_index(1));
        acc_PtA = Vtotal^2/R*N1_hat;
        acc_PtB = Vtotal^2/R*N2_hat;
        wptsX_out =[wptsX_out; PtA(1) L1_hat(1)*Vtotal acc_PtA(1); PtB(1) L2_hat(1)*Vtotal acc_PtB(1); wptsX(xy_index(1)+2:end,:)];
        wptsY_out =[wptsY_out; PtA(2) L1_hat(2)*Vtotal acc_PtA(2); PtB(2) L2_hat(2)*Vtotal acc_PtB(2); wptsY(xy_index(2)+2:end,:)];
        time_wptsX_out = [time_wptsX_out, norm(PtA-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/Vtotal+time_wptsX(xy_index(1)), ...
            norm(PtA-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/Vtotal+abs((psi_L2-psi_L1)/turn_rate_act)+time_wptsX(xy_index(1)), time_wptsX(xy_index(1)+2:end)];
        time_wptsY_out = [time_wptsY_out, norm(PtA-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/Vtotal+time_wptsY(xy_index(2)),  ...
            norm(PtA-[wptsX(xy_index(1),1);wptsY(xy_index(2),1)])/Vtotal+abs((psi_L2-psi_L1)/turn_rate_act)+time_wptsX(xy_index(2)), time_wptsY(xy_index(2)+2:end)];
        % Update the xy_index
        xy_ind_out = xy_index + 2;
        num_added = 2; % Added two waypoints
    end
end % End of check for small angle turn...