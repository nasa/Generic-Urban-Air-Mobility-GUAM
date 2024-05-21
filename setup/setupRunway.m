function Out = setupRunway(SimIn)
% Runway
%
% $Id$

  % http://www.airnav.com/airport/KEDW
  % Latitude  = (  34 +54.977463/60)*SimIn.Units.deg;
  % Longitude = (-117 -51.745132/60)*SimIn.Units.deg;
  % Elevation = 2281.9**SimIn.Units.ft;
  % Heading   = 238**SimIn.Units.deg;% True heading

  %Placeholder values
  Out.Airport   = 'KLFI';
  Out.Runway    = '08'; 
  Out.TimeZone = '-5';% UTC
  Out.Heading = (80 - 10)*SimIn.Units.deg;
  Out.Lat= ( 34  +4.658833/60)*SimIn.Units.deg;
  Out.Lon     = (-76 -22.583667/60)*SimIn.Units.deg;
  %Out.AltMSL  = 8.6*SimIn.Units.ft;
  Out.Alt     = 0 *SimIn.Units.ft;

  Out.UseFG = 0;

end