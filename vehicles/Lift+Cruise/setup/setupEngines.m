function Out = setupEngines(SimIn)
% L+C engines
% Total of 9 rotor/propeller actuators
% 

nENG = SimIn.numEngines;

wn   = 4*pi; % rad/s
freq = wn / (2*pi); % Hz
zeta = 0.7;
rl   = 100.0; % rad/s
if SimIn.fmType==ForceMomentEnum.SFunction % Sfunction enumeration
%     RotorLim  = [-0.001;  350.0]; % Rotor limits (rad/s)
%     PropLim  =  [-0.001;  350.0]; % Propeller limits (rad/s)
    RotorLim  = [0;  350.0]; % Rotor limits (rad/s)
    PropLim  =  [0;  350.0]; % Propeller limits (rad/s)
elseif SimIn.fmType ==ForceMomentEnum.Polynomial % Polynomial enumeration
%     RotorLim  = [-0.001;  1600] * 2*pi/60; % Rotor limits (rad/s)
%     PropLim  =  [-0.001;  2000] * 2*pi/60; % Propeller limits (rad/s)
    RotorLim  = [0;  1600] * 2*pi/60; % Rotor limits (rad/s)
    PropLim  =  [0;  2000] * 2*pi/60; % Propeller limits (rad/s)
end


%% Outputs
Out.nENG      = nENG;
Out.BW        = repmat(freq,nENG,1); % (Hz)
Out.Zeta      = repmat(zeta,nENG,1);
Out.delay     = 0;% (sec) Transport delay
Out.RateLim   = repmat(rl,nENG,1); % (rad/s)

Out.PosLim_hi = [RotorLim(2)*ones(8,1);PropLim(2)]; % (rad/s)
Out.PosLim_lo = [RotorLim(1)*ones(8,1);PropLim(1)]; % (rad/s)
Out.Bias      = SimIn.IC.bias.Engines;
