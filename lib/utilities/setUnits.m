function u = setUnits(distUnit,massUnit)
% SETUNITS Compute and establish unit conversion constants.
%
%       u = setUnits(distUnit,massUnit);
%
%           distUnit = name of base distance unit
%           massUnit = name of base mass unit
%
%           u = returned structure of unit conversion scalars
%
%	The allowable distance (base) units are 'm', 'cm', 'ft', and 'in'.
%	The allowable mass (base) units are 'kg', 'gram', 'slug', 'slinch', and
%       'lbm', where the slinch = 12 slugs.
%   Not all combinations are allowed; both must be either metric or English.
%
%   If distUnit is given as 'SI', then distUnit='m' and massUnit='kg'.  If
%   neither parameter is given, then the default is 'SI'.
%
%   If massUnit is not given, the default depends on the distUnit...
%       for distUnit = 'm' or 'SI',	 default massUnit = 'kg'
%       for distUnit = 'cm',         default massUnit = 'gram'
%       for distUnit = 'ft',         default massUnit = 'slug'
%       for distUnit = 'in',         default massUnit = 'slinch'
% 
% Example 1:
%   To convert from a supplied unit to the base unit simply multiply by the
%   unit conversion:
%   Requirments give wingspan, b as 10 meters. A proper usage of this
%   units structure, u would be:
%       b = 10*u.m;
%   Notice the unit symbol documents the value.  If base distance units are
%   meters than b = 10.  If base distants units are feet then b =
%   10/0.3048;
% 
% Example 2:
%   To convert from the base unit to a particular unit then divide by the
%   unit converison.  Given b = 10 meters, the base units are meters (u.m =
%   1), and we wish to know the value of b in feet. The proper usage would
%   be: b/u.ft
% 
%%  Defining constants
% 
% The documents referenced below are from NIST, the agency charged by Federal
% statute with defining the units of measurement for the United States.  
% It does so in a manner consistent with the various international standards
% agencies and bodies (BIPM, CIPM, and CGPM), under the auspices of the 
% 1875 Treaty of the Meter.
% 
% The documents are available at https://www.nist.gov/pml/productsservices/special-publications-tutorials  
% (at the bottom of the page you will see links to the documents).
% 
% SP330 is a brochure which gives the definitions for the SI units.
% 
% SP811 is a usage guide primarily aimed at formal technical usage of the SI,
% but it does list conversions for a number of non-SI units.  At the bottom 
% of page 53 are notes (22 and 23) which give the exact conversions of the 
% pound (to the kilogram) and the pound-force (to the Newton).
% 
% Both of the preceding documents are current (2008 edition).
% 
% SP447 is more of a historical document which gives some of the historical 
% basis for the units.  The current definitions for the yard (0.9144 meter)
% and the pound (0.45359237 kg) are given at the bottom of page 20.  Some
% of the earlier physical standards are shown and discussed on pages 22-25;
% the mass standards were either based on the kilogram or the pound;
% there is no mention of the slug.  A more thorough discussion of the 1959
% (current) definition of the yard and pound is given in Appendix 5.
%
% We will use an operational definition of the slug based on the relationship
% lbf = lbm*g0 = slug*ft/s^2, where g0 is the standard acceleration of free
% fall.  This is consistent with the approximate value of the slug given on
% page 54 of SP811.

%   2010-07-06  S.Derry     original version
%   2012-09-14  S.Derry     clarify sources of definitions
%   2016-02-17  E. Heim     Added examples
%   2023-03-21  M. Acheson  Updated location of references (docs are publicly released)


defineFoot = 0.3048;        % foot = 0.3048 meter (SP811 pg.49)
%   Also see SP811 Section B.6, pg.42.
definePound = 0.45359237;   % lbm = 0.45359237 kg (SP811 pg.53 and Note 22)
defineG = 9.80665;          % g (g0) = 9.80665 m/s^2 (SP811 pg.53 Note 23)
%   Also see SP811, pg.45 (acceleration of free fall, standard) and SP330 pg.52.

defineKRankine = 1.8;       % Kelvin = 1.8 degree Rankine
zeroCelsius = 273.15;       % zero Celsius = 273.15 K
zeroFahrenheit = zeroCelsius * defineKRankine - 32.0;	% zero Fahrenheit = 459.67 deg R

defineNM = 1852.0;          % nautical mile = 1852 meter (SP811 pg.52 and Note 20)
%   Also see SP811 Table 9, pg.10.

defineAtmos = 101325.0;     % standard atmosphere = 101325 Pa (SP811 Table 11 pg.11)
defineMmHg = 133.3224;      % mm of mercury = approx. 133.3224 Pa (SP811 pg.52)

%%  Determine base units by parameter

%   default and SI cases

if nargin < 1
    distUnit = 'SI';
end
if strcmpi(distUnit, 'SI')
    distUnit = 'm';
    massUnit = 'kg';
end

%   distance unit parameter

if strcmpi(distUnit, 'm')       % meter
    metric = 1;
    m = 1.0;
    cm = 0.01;
    defaultMass = 'kg';

elseif strcmpi(distUnit, 'cm')  % centimeter
    metric = 1;
    m = 100.0;
    cm = 1.0;
    defaultMass = 'gram';

elseif strcmpi(distUnit, 'ft')  % foot
    metric = 0;
    ft = 1.0;
    in = 1.0 / 12.0;
    defaultMass = 'slug';

elseif strcmpi(distUnit, 'in')  % inch
    metric = 0;
    ft = 12.0;
    in = 1.0;
    defaultMass = 'slinch';
    
else
    error(['Invalid distance unit "' distUnit '".']);
end

if metric       % define English units with metric base
    ft = defineFoot * m;
    in = ft / 12.0;
else            % define metric units with English base
    m = ft / defineFoot;
    cm = m / 100.0;
end

%   Note that (for brevity) the code in places assumes that the second = 1.

g0 = defineG * m;       % standard free fall (m/s^2)

%   mass unit parameter

if nargin < 2                   % default mass unit
    massUnit = defaultMass;     % determined by base distance
end

if strcmpi(massUnit, 'kg')      % kilogram
    if ~metric
        error(['Cannot mix metric "kg" with English "' distUnit '".']);
    end
    kg = 1.0;
    gram = 0.001;

elseif strcmpi(massUnit, 'gram')	% gram
    if ~metric
        error(['Cannot mix metric "gram" with English "' distUnit '".']);
    end
    kg = 1000.0;
    gram = 1.0;

elseif strcmpi(massUnit, 'slug')	% slug
    if metric
        error(['Cannot mix English "slug" with metric "' distUnit '".']);
    end
    slug = 1.0;
    slinch = 12.0;
    lbm = slug * (ft/g0);   % lbm*g0 = slug*ft/s^2 = lbf

elseif strcmpi(massUnit, 'slinch')	% slinch
    if metric
        error(['Cannot mix English "slinch" with metric "' distUnit '".']);
    end
    slug = 1.0 / 12.0;
    slinch = 1.0;
    lbm = slug * (ft/g0);   % lbm*g0 = slug*ft/s^2 = lbf

elseif strcmpi(massUnit, 'lbm')     % pound mass
    if metric
        error(['Cannot mix English "lbm" with metric "' distUnit '".']);
    end
    lbm = 1.0;
    slug = lbm * (g0/ft);   % lbm*g0 = slug*ft/s^2 = lbf
    slinch = 12.0 * slug;
    
else
    error(['Invalid mass unit "' massUnit '".']);
end

if metric       % define English units with metric base
    lbm = definePound * kg;
    slug = lbm * (g0/ft);   % lbm*g0 = slug*ft/s^2 = lbf
    slinch = 12.0 * slug;
else            % define metric units with English base
    kg = lbm / definePound;
    gram = 0.001 * kg;
end


%%  Establish base units

%   distance

u.m = m;        % meter
u.cm = cm;      % centimeter
u.ft = ft;      % foot
u.in = in;      % inch

%   mass

u.kg = kg;      % kilogram
u.gram = gram;  % gram (not abbreviated to avoid confusion with g0)
u.lbm = lbm;    % pound mass (not lb to avoid confusion with lbf)
u.slug = slug;  % slug
u.slinch = slinch;  % slinch == lbf / (in/s^2)

%   temperature

if metric       % stay consistent with specified unit system
    u.K = 1.0;                      % Kelvin
    u.degR = 1.0 / defineKRankine;	% degree Rankine
else
    u.K = defineKRankine;           % Kelvin
    u.degR = 1.0;                   % degree Rankine
end

u.zeroC = zeroCelsius;      % offset from Kelvin to Celsius
u.zeroF = zeroFahrenheit;	% offset from Rankine to Fahrenheit

%   others

u.s = 1.0;      % second    (must be 1 for all systems)
u.rad = 1.0;    % radian



%%  Define derived units

%   distance

u.km = 1000 * u.m;          % kilometer = 1000 m
u.mm = 0.001 * u.m;         % millimeter = m / 1000
u.nmile = defineNM * u.m;	% nautical mile = 1852 m
u.smile = 5280 * u.ft;      % statute mile = 5280 feet

%   time

u.hour = 3600 * u.s;        % hour = 3600 s

%   speed

u.knot = u.nmile / u.hour;  % knot = nautical mile / hour

%   acceleration

u.g0 = g0;                  % standard free fall

%   area

u.m2 = m * m;       % square meter
u.ft2 = ft * ft;    % square foot
u.in2 = in * in;    % square inch

%   force

u.N = kg * m;       % Newton = kg * m / s^2
u.dyne = gram * cm; % dyne = gram * cm / s^2
u.lbf = lbm * g0;   % pound force = pound mass * standard free fall
u.poundal = lbm * ft;   % poundal = pound mass * ft/s^2

%   pressure

u.Pa = u.N / u.m2;      % Pascal = Newton / square meter
u.mbar = 100 * u.Pa;    % millibar = 100 Pascals
u.psf = u.lbf / u.ft2;  % pound / square foot
u.psi = u.lbf / u.in2;  % pound / square inch
u.atmos = defineAtmos * u.Pa;   % standard atmosphere
u.mmHg = defineMmHg * u.Pa;     % millimeter of mercury
u.inHg = u.mmHg * u.in / u.mm;  % inch of mercury

%   energy

u.J = u.N * m;          % Joule = Newton meter
u.erg = u.dyne * u.cm;  % erg = dyne cm
u.ftlbf = u.lbf * ft;   % foot pound

%   power

u.W = u.J / u.s;        % Watt = Joule / s
u.hp = 550 * u.ftlbf / u.s;	% horsepower = 550 ft*lbf/s

%   angle

u.deg = pi / 180;           % degree (of arc)
u.arcmin = u.deg / 60;      % arc minute
u.arcsec = u.arcmin / 60;   % arc second

