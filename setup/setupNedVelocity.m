function [Vel_bEh] = setupNedVelocity(SimIn, Q_h2b)
%function [Vel_bEh] = setupNedVelocity(SimIn)
%
% Define Vel_bEh based on whether gamma or alpha is a state
%
% Inputs:
%   SimIn:  SimIn input structure with initial conditions
%   Q_h2b:  Quaternion from NED to Body
%

IC  = SimIn.IC;

% Initial EOM states (position, attitude, etc)
V     = IC.Vgrnd;
gamma = IC.gamma;
chi   = IC.track;
alpha = IC.alpha;
beta  = IC.beta;

%Determine whether alpha or gamma is a state and define Vel_bEh accordingly

have_trim = isfield(SimIn,'Trim');
if have_trim
  if SimIn.Trim.States.gamma.perturb && SimIn.Trim.States.alpha.perturb
    error('Both alpha and gamma cannot be perturbed states');
  end
end

if have_trim && strcmp(SimIn.Trim.Set.Vel_bEh,'VAB')
  Vel_bEh = Qtrans(Qinvert(Q_h2b),V*[cos(alpha)*cos(beta); sin(beta);sin(alpha)*cos(beta)]); %assume no wind
else
  Vel_bEh = V*[cos(gamma)*cos(chi); cos(gamma)*sin(chi); -sin(gamma)];% initial NED velocity
end

end

