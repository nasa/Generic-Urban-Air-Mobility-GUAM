function Out = setup_control(SimIn)

% Setup script for Lift Plus Cruise baseline control
% Jacob Cook
% 2/21/2020

d2r = SimIn.Units.deg;

% read in the refrence trajectory
refInputs = SimIn.RefInputs;

% load the trim conditions for the aircraft
if SimIn.fmType == ForceMomentEnum.SFunction
    Out.trim = load('trim_table.mat');
elseif SimIn.fmType == ForceMomentEnum.Polynomial
    % Out.trim = load('trim_table_poly.mat'); % Ben's original file
    Out.trim = load('trim_table_Poly_ConcatVer4p0.mat'); %Trim_ver3p0
end

% Flight path angle gains

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% propellers are number from left to right, then front to back
% 
%                 
%                          / \
%                  (1) (2) | | (3) (4)
%                 ,-------------------,
%                 '-------------------'
%                  (5) (6) | | (7) (8)
%                          | |
%                       ,-------,
%                       '-------'
%                          (9)
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% surface allocation
%                            
%                          / \
%                          | |  
%                 ,-------------------,
%                 '-1---------------2-'
%                          | |  
%                          | |
%                       ,---|---,
%                       '3--|--4'
%                           5 
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                                1  2  3  4  5 
%                              -----------------

Out.surface_alloc.alloc_flap  = [ 1  1  0  0  0 ]';
Out.surface_alloc.alloc_ail  = [-1  1  0  0  0 ]';
Out.surface_alloc.alloc_elev = [ 0  0  1  1  0 ]';
Out.surface_alloc.alloc_rud  = [ 0  0  0  0  1 ]';


% zero out the R for now
Out.trim.R = 0;

Vel_bIc_0 = refInputs.initialVelocity;
track     = refInputs.initialTrack;

% Use the horizontal component to set the initial 
% conditions of the controls for now. Eventually 
% the flight path angle will be used
ubar0 = Vel_bIc_0(1);
wbar0 = Vel_bIc_0(3);
for ii = 1:25
    if length(Out.trim.WH) > 1 
        XU0(ii,1) = interp2(Out.trim.WH, Out.trim.UH, squeeze(Out.trim.XU0_interp(ii,:,:)), wbar0, ubar0,'linear',Out.trim.XU0_interp(ii,1,2));
    else
        XU0(ii,1) = interp1(Out.trim.UH, Out.trim.XU0_interp(ii,:), ubar0, 'linear','extrap');
    end
end
Out.XU0 = XU0;

% Set the initial conditions
Out.IC.Vtot0       = norm(XU0(1:3,1));
Out.IC.Vtot0       = norm(Vel_bIc_0);
Out.IC.Vel_bIc_0   = Vel_bIc_0;
Out.IC.alpha0      = atan2(XU0(3),XU0(1));
Out.IC.beta0       = 0;
Out.IC.gamma0      = -atan2(Vel_bIc_0(3), norm(Vel_bIc_0(1:2)));
Out.IC.chi0        = track;
Out.IC.vb0         = XU0(1:3,1);
Out.IC.om0         = XU0(4:6,1);
Out.IC.ab0         = XU0(7:9,1);
Out.IC.eta0        = XU0(10:12,1);

delf0       = XU0(13,1);
dela0       = XU0(14,1);
dele0       = XU0(15,1);
delr0       = XU0(16,1);
om_rotor0   = XU0(17:24,1);
om_pusher0  = XU0(25,1);

Out.IC.surf0 = delf0*Out.surface_alloc.alloc_flap + ...
               dela0*Out.surface_alloc.alloc_ail + ...
               dele0*Out.surface_alloc.alloc_elev + ...
               delr0*Out.surface_alloc.alloc_rud;

Out.IC.rotor0 = [om_rotor0; om_pusher0];

% setup the baseline_1 adaptive gains
Out.Adaptive = setupAdaptive(SimIn);

end
