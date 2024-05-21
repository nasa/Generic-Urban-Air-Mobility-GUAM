%
% setupVariantStruct.m
%
% Script used to create the 'userStruct.variants' variable that is used in the
% process of setting the various 'type' variables in the SimIn structure
% (see setupTypes.m).  This script allows for user-specified integer values
% into the appropriate fields.  Any user specified integer values are cast
% to the appropriate enumeration in setupTypes.m.  If a field is not
% specified, a default enumeration value is assigned.  If a '-1' value is
% assigned to the field, that will cause the menu selection function to be
% invoked for that particular 'type' in setupTypes.m
%
% For example, if the only user-specified value was provided for the
% vehicleType field:
%
% userStruct.variants.vehicleType = 1
%
% The resulting userStruct.variants would look like:
% userStruct.variants = 
% 
%   struct with fields:
% 
%     vehicleType: 1
%         expType: DEFAULT
%       atmosType: US_STD_ATMOS_76
%        turbType: None
%        ctrlType: BASELINE
%    refInputType: EUTL_TRAJ
%         actType: FirstOrder
%        propType: None
%          fmType: SFunction
%         eomType: STARS
%      sensorType: None
%
% Where the user-specified fields keep the original values, and the
% unspecified ones are assigned the default enumeration values.
%
% The resulting entry in SimIn for vehicleType would look like this:
%   SimIn.vehicleType = VehicleEnum.LiftPlusCruise
%
% Any user-specified field set to -1 for menu selection is removed from the
% userStruct.variants variable.
%

bUserExist = exist('userStruct','var');
if bUserExist
  bExist = isfield(userStruct, 'variants');
  disp('userStruct exists')
else
  bExist = false;
  disp('userStruct does not exist');
end

if bExist
  disp('userStruct.variants exists');
else
  disp('userStruct.variants does not exist');
end

% common choices
if ~bExist || (bExist && ~isfield(userStruct.variants,'vehicleType'))
  % use subscript in case userStruct.variants isn't specified initially
  userStruct.variants.vehicleType = VehicleEnum.LiftPlusCruise;
end
if ~bExist || (bExist && ~isfield(userStruct.variants,'expType'))
  userStruct.variants.expType = ExperimentEnum.DEFAULT;
elseif (bExist && (userStruct.variants.expType == -1))
  disp('User requested selection menu for experiment type');
  userStruct.variants = rmfield(userStruct.variants,'expType');
end
if ~bExist || (bExist && ~isfield(userStruct.variants,'atmosType'))
  userStruct.variants.atmosType = AtmosphereEnum.US_STD_ATMOS_76;
elseif (bExist && (userStruct.variants.atmosType == -1))
  disp('User requested selection menu for atmosphere type');
  userStruct.variants = rmfield(userStruct.variants,'atmosType');
end
if ~bExist || (bExist && ~isfield(userStruct.variants,'turbType'))
  userStruct.variants.turbType = TurbulenceEnum.None;
elseif (bExist && (userStruct.variants.turbType == -1))
  disp('User requested selection menu for turbulence type');
  userStruct.variants = rmfield(userStruct.variants,'turbType');
end

% root dir of the sim
rootDir = getenv("GVSActiveRootDir");

% add vehicle-specific path
switch userStruct.variants.vehicleType
  case VehicleEnum.LiftPlusCruise
    vehDir = 'vehicles/Lift+Cruise';
  case VehicleEnum.Quad6
    vehDir = 'vehicles/Quad6';
  case VehicleEnum.GenTiltRotor
    vehDir = 'vehicles/GenTiltRotor';
  case VehicleEnum.GenTiltWing
    vehDir = 'vehicles/GenTiltWing';
  case VehicleEnum.GL10
    vehDir = 'vehicles/GL-10';
  case VehicleEnum.LA8
    vehDir = 'vehicles/LA-8';
  otherwise
    error('UNRECOGNIZED VEHICLE TYPE')
end
path(genpath(vehDir), path);
SimIn.vehiclepath = sprintf('%s/%s/', rootDir, vehDir);

% vehicle-specific choices
switch userStruct.variants.vehicleType
  case VehicleEnum.LiftPlusCruise
    % default Lift+Cruise selections
    userStruct.variants = setupDefaultChoices(userStruct.variants, bExist);
  case VehicleEnum.LA8
    % default LA-8 selections
    userStruct.variants = setupDefaultChoices(userStruct.variants, bExist);
  case VehicleEnum.Quad6
  case VehicleEnum.GenTiltRotor
  case VehicleEnum.GenTiltWing
  case VehicleEnum.GL10
    error('VEHICLE CURRENTLY NOT IMPLEMENTED');
  otherwise
    error('UNRECOGNIZED VEHICLE TYPE')
end

% clear vehDir rootDir bExist;
clear rootDir bExist;

