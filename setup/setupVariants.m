disp('Variant setup');

% setup common variant conditions
setupVehicleVariants;
setupAtmosphereVariants;
setupTurbulenceVariants;
setupExperimentVariants;

% set model-specific variant conditions
setupControllerVariants;
setupRefInputsVariants;
setupActuatorVariants;
setupPropulsionVariants;
setupForceMomentVariants;
setupEOMVariants;
setupSensorVariants;

