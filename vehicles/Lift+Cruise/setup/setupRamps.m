function Out = setupRamps(SimIn)

s   = SimIn.Units.s;
d2r = SimIn.Units.deg;

% ******** Ramp 1 *********************
% velocity (uvw) step times
Out.Vel_startTime1 = [0; 0; 0] * s; % s

% velocity (uvw) initial values
Out.Vel_initialValue1 = [0; 0; 0;]; % ft/s

% velocity (uvw) slope value
Out.Vel_slope1 = [0; 0; 0;]; % ft/s

% ******** Ramp 2 *********************
% velocity (uvw) step times
Out.Vel_startTime2 = [0; 0; 0] * s; % s

% velocity (uvw) initial values
Out.Vel_initialValue2 = [0; 0; 0;]; % ft/s

% velocity (uvw) slope value
Out.Vel_slope2 = [0; 0; 0;]; % ft/s

% ******** Ramp 3 *********************
% velocity (uvw) step times
Out.Vel_startTime3 = [0; 0; 0] * s; % s

% velocity (uvw) initial values
Out.Vel_initialValue3 = [0; 0; 0;]; % ft/s

% velocity (uvw) slope value
Out.Vel_slope3 = [0; 0; 0;]; % ft/s

% ******** Ramp 4 *********************
% velocity (uvw) step times
Out.Vel_startTime4 = [0; 0; 0] * s; % s


% velocity (uvw) initial values
Out.Vel_initialValue4 = [0; 0; 0;]; % ft/s

% velocity (uvw) slope value
Out.Vel_slope4 = [0; 0; 0;]; % ft/s

% ******************************************

% track step time
Out.Track_stepTime = 10 * s; %s

% track initial value
Out.Track_initialValue = 0 * d2r; % rad

% track final value
Out.Track_finalValue = 0 * d2r; % rad

end

