function Out = setupActuators(SimIn)
% L+C actuators
% Total of 4 physical actuators: Aileron Flap Elevator Rudder.  Left and right
% ailerons, flaps, and elevators will be commanded together.
% 

deg = SimIn.Units.deg;

nCS = SimIn.numSurfaces;

wn   = 20.0; % rad/s
freq = wn / (2*pi); % Hz
zeta = 0.7;
rl   = 100.0; % rad/s

EleLim  = [-30.0;  30.0] * deg; % Elevator bounds (degrees)
AilLim  = [-30;  30] * deg;     % Aileron limits (degrees)
FlpLim  = [-30;  30] * deg;     % Flap limits (degrees)
RudLim  = [-30;  30] * deg;     % Rudder limits (deg)

%% Outputs
Out.nCS       = nCS;
Out.BW        = repmat(freq,nCS,1); % (Hz)
Out.Zeta      = repmat(zeta,nCS,1);
Out.delay     = 0;% (sec) Transport delay
Out.RateLim   = repmat(rl,nCS,1); % (rad/s)

Out.PosLim_hi = [AilLim(2)*ones(2,1);EleLim(2)*ones(2,1);RudLim(2)]; % (rad)
Out.PosLim_lo = [AilLim(1)*ones(2,1);EleLim(1)*ones(2,1);RudLim(1)]; % (rad)
Out.Bias      = SimIn.IC.bias.Surfaces;