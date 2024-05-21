function [pos_i, vel_i, acc_i, chi, chid, time_violate_flag] = evalSegments(wptsX, wptsY, wptsZ, time_wptsX, time_wptsY, time_wptsZ, time)
% function Script: evalSegments.m

% *************************************************************************
% Description: This script is used to interrogate one (or more) particular 
% segments of cells (wayponts and time_wpts) for position, velocity and
% accelerations.  This requires computation of control points and then
% using these to interrogate an individual point.
%
% In:
%   wptsX, wptsY, wptsZ:  An N x M array where N = # of segments(waypoints) 
%               and M = number of derivatives (e.g., pos vel accel)
%   time_wptsX (Y,Z): Row vector: doubles with time points for each segment
%               (if input is from constant block with "Treat vector as 1-D"
%               checked, then uncheck to preserve row vector input). NOTE:
%               only first and last times must be consistant across X,Y,Z
%   time:       column vector of times at which to interrogate the
%               bezier information
%
% Out:
%   pos_i:              Bezier curve evaluated for position at the input time
%   vel_i:              Bezier curve evaluated for velocity at the input time
%   accel_i:            Bezier curve evaluated for acceleration at the input time
%   chi:                Computed heading frame angle
%   chid:               Computed heading frame angle derivative
%   time_violate_flag:  Flag denotes one or more times outside time_wpts,
%                       -1 => time less than start time,
%                       1 => time greater than stop time
%
% Created:
%   9/9/2022
%   Michael Acheson
%   NASA Langley Research Center, D316
%   Email: michael.j.acheson@nasa.gov
%
% Mods:
%   DD Month YYYY - Name : Description
%   10/14/2022 MJA Modified script to be capable of handling input time as
%   a vector (not just a scalar.)  This enables its' use for other than just
%   trajectory lookup (e.g. at current sim time) such as for use with DDP.
%   Note, the time_wpts (X,Y & Z)vectors are assumed to have identical start 
%   and end times but may have differing number of wpts.  Any input time
%   vector element that asks for a time lookup earlier than the first
%   time_wpt (or after the last time) is adjusted to "hold" the corresponding
%   first/last wpt time.

% *************************************************************************

% Check time vector length against mask size
tlen = length(time); % Determine number of input times

% Hardcode wpt dimensions (x,y,z & num derivatives)
N_dim = 3; % X, Y & Z axes
N_der = 3; % position, velocity and acceleration derivatives

% Check if any time is before or after the minimum/maximum wpts time 
% NOTE: the initial and final wpt_timeX, wpt_timeY, and wpt_timeZ times 
% are assumed to be equivalent
time_violate_flag = 0; % Flag used to output evaluation time outside time_wpts

time_lt_min = time < time_wptsX(1);
if any(time_lt_min)
   time(time_lt_min) = ones(sum(time_lt_min),1)*time_wptsX(1); 
   time_violate_flag = -1; % Specify time violation below first time_wpts vector component
end

time_grt_max = time > time_wptsX(end);
if any(time_grt_max)
   time(time_grt_max)   = ones(sum(time_grt_max),1)*time_wptsX(end);
   time_violate_flag    = 1; % Specify time violation above last time_wpts vector component
end

% Initialize outputs 
pos_i   = zeros(3,tlen);
vel_i   = zeros(3,tlen);
acc_i   = zeros(3,tlen);

% Perform for each axis
time_wpts = zeros(1,max([length(time_wptsX), length(time_wptsY), length(time_wptsZ)]));
waypoints = zeros(max([size(wptsX,1), size(wptsY,1), size(wptsZ,1)]),max([size(wptsX,2), size(wptsY,2), size(wptsZ,2)]));
for n_dim = 1:N_dim
    switch n_dim
        case 1
            waypoints = wptsX;
            time_wpts = time_wptsX;
        case 2
            waypoints = wptsY;
            time_wpts = time_wptsY;
        case 3
            waypoints = wptsZ;
            time_wpts = time_wptsZ;
    end
    
    % Locate the time of interest
    [ind_seg] = discretize(time, time_wpts);
    uniq_ind_seg = unique(ind_seg,'stable'); % Determines number of unique indices (and retains sort order)
    num_uniq_ind = length(uniq_ind_seg);
    
    for loop = 1:num_uniq_ind % Cycle thru each unique bin (to compute control points and evaluate)
        wpts        = [waypoints(uniq_ind_seg(loop),1:N_der); waypoints(uniq_ind_seg(loop)+1,1:N_der)];
        tint        = time_wpts(uniq_ind_seg(loop): uniq_ind_seg(loop)+1);
        loop_ind    = (ind_seg == uniq_ind_seg(loop));
        % Compute the control points for current dimension
        ctl         = nan(N_der*2,N_der);
        ctl(:,1)    = interpHermBern(wpts,tint);
        for n_der = 1:N_der-1
            ctl_1 = diffBernControl(ctl(:,1),tint,n_der);
            ctl(:,n_der+1) = [ctl_1;nan(n_der,1)];
        end

        % Using the control points, evaluate the curve at the prescribed time
        Q1              = ctl(:,1);
        Q1              = Q1(~isnan(Q1));
        pos_i(n_dim, loop_ind) = evalBernPoly(Q1,time(loop_ind),tint);
        Q2              = ctl(:,2);
        Q2              = Q2(~isnan(Q2));
        vel_i(n_dim, loop_ind) = evalBernPoly(Q2,time(loop_ind),tint);
        Q3              = ctl(:,3);
        Q3              = Q3(~isnan(Q3));
        acc_i(n_dim, loop_ind) = evalBernPoly(Q3,time(loop_ind),tint);
    end

end % End of for n_dim = 1:N_dim

% Find heading frame angle and derivative
chi     = atan2(vel_i(2,:),vel_i(1,:))';
tempval = vel_i(1,:).^2 + vel_i(2,:).^2;
delfdelx = -vel_i(2,:)./tempval;
delfdely =  vel_i(1,:)./tempval;
chid = (delfdelx.*acc_i(1,:))' + (delfdely.*acc_i(2,:))';

% Check for any NaNs in chid, if they occur, set chid vector element to zero
chid_nan_ind = isnan(chid);
if any(chid_nan_ind)
    chid(chid_nan_ind) = zeros(sum(chid_nan_ind),1);
end