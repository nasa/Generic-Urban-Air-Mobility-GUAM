function out = setupAtmosphere(SimIn)
%
%   setup atmospheric properties
% $Id: setupAtmosphere.m 

% Modifications
% 2016-06-13 14:52:49Z jkcarbon: Initial version
% 2/28/2023 MJA: Updated references, standard atmosphere properties (temp, 
% pressure and gamma) from:
% "Aircraft Performance Flight Testing", Technical Information Handbook September 2000, AFFTC-TIH-99-01

  u = SimIn.Units;

  R     = 8.31432e3;% (N*m/kmol*K) Gas constant
  M0    = 28.9644;% (kg/kmol)
  T0    = 288.15;% (K) sea-level temperature
  P0    = 1.013250e5;% (N/m^2) sea-level atmospheric pressure
  gamma = 1.400;% (n/a) Ratio of specific heats for dry air

  out.gamma = gamma;
  out.Ps0  = P0*u.N/u.m^2;
  out.T0   = T0*u.K;
  out.rho0 = P0*M0/(R*T0)*u.kg/u.m^3; % Ideal gas law


  %%   Deviation from standard atmosphere
  out.deltaT = 0.0 *u.degR;%(°R) Temperature deviation from standard 

end