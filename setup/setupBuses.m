disp('Bus setup');

% setup bus objects
clear ControlBus EnvironmentBus EomBus ForceMomentBus SurfaceActuatorBus SensorBus BUS_REF_INPUT;

% define the bus objects
ControlBus(true, SimIn.numEngines, SimIn.numSurfaces);
EnvironmentBus;
EomBus;
ForceMomentBus(true, SimIn.numEngines);
RefInputBus;
PowerSystemInBus(true, SimIn.numEngines, SimIn.numSurfaces);
PowerSystemOutBus(true, SimIn.numEngines, SimIn.numSurfaces);
PropActuatorBus(true, SimIn.numEngines);
SurfaceActuatorBus(true, SimIn.numSurfaces);
SensorBus;
TrimInputBus(true, SimIn.Trim.numEngines, SimIn.Trim.numSurfaces);
FailureBusSurf(true,0, SimIn.numSurfaces); 
FailureBusEng(true,SimIn.numEngines, 0); 

% must go after all other vehicle buses are defined
VehicleOutBus;
% must go after all other buses defined
SimOutBus;

% remove default return variable
clear ans;
