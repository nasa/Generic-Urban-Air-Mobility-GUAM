function Out = setupDoublets(SimIn)

s   = SimIn.Units.s;
d2r = SimIn.Units.deg;

% velocity (uvw) step times
Out.Vel_stepTime = [0; 0; 5] * s; % s

% velocity (uvw) initial values
Out.Vel_initialValue = [0; 0; 0;]; % ft/s

% velocity (uvw) final value
Out.Vel_finalValue = [0; 0; -10;]; % ft/s

% track step time
Out.Track_stepTime = 10 * s; %s

% track initial value
Out.Track_initialValue = 0 * d2r; % rad

% track final value
Out.Track_finalValue = 0 * d2r; % rad

end

