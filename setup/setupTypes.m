function [Out, vs] = setupTypes(SimIn, variantStruct)
  arguments
    SimIn struct
    variantStruct struct = []
  end

  % select experiment
  if isfield(variantStruct,'expType')
    m = enumeration('ExperimentEnum');
    SimIn.expType = m(variantStruct.expType);
  else
    SimIn.expType = selectExperimentType;
  end

  % select vehicle model
  if isfield(variantStruct,'vehicleType')
    m = enumeration('VehicleEnum');
    SimIn.vehicleType = m(variantStruct.vehicleType);
  else
    SimIn.vehicleType = selectVehicleType;
  end

  % select atmosphere model
  if isfield(variantStruct,'atmosType')
    m = enumeration('AtmosphereEnum');
    SimIn.atmosType = m(variantStruct.atmosType);
  else
    SimIn.atmosType = selectAtmosphereType;
%    SimIn.atmosType = AtmosphereEnum.US_STD_ATMOS_76;
  end

  % select disable/enable turbulence model
  if isfield(variantStruct,'turbType')
    m = enumeration('TurbulenceEnum');
    SimIn.turbType = m(variantStruct.turbType);
  else
    SimIn.turbType = selectTurbulenceType;
%    SimIn.turbType = TurbulenceEnum.None;
  end

  % select flight mode
  %SimIn.mode = selectFlightMode;
  SimIn.mode = FlightModeEnum.CRUISE;

  % select controller model
  if isfield(variantStruct,'ctrlType')
    m = enumeration('CtrlEnum');
    SimIn.ctrlType = m(variantStruct.ctrlType);
  else
    SimIn.ctrlType = selectControllerType;
  end

  % select ref input
  if isfield(variantStruct,'refInputType')
    m = enumeration('RefInputEnum');
    SimIn.refInputType = m(variantStruct.refInputType);
  else
    SimIn.refInputType = selectRefInputType;
  end

  % select control surface actuator model
  if isfield(variantStruct,'actType')
    m = enumeration('ActuatorEnum');
    SimIn.actType = m(variantStruct.actType);
  else
    SimIn.actType = selectActuatorType(SimIn.ctrlType);
  end
  
  % select propulsion model
  if isfield(variantStruct,'propType')
    m = enumeration('PropulsionEnum');
    SimIn.propType = m(variantStruct.propType);
  else
    SimIn.propType = selectPropulsionType(SimIn.ctrlType);
  end
  
  % select aero/propulsion force/moment model
  if isfield(variantStruct,'fmType')
    m = enumeration('ForceMomentEnum');
    SimIn.fmType = m(variantStruct.fmType);
  else
    SimIn.fmType = selectForceMomentType;
  end
  
  % select EOM
  if isfield(variantStruct,'eomType')
    m = enumeration('EOMEnum');
    SimIn.eomType = m(variantStruct.eomType);
  else
    SimIn.eomType = selectEOMType;
  end
  
  % select sensor model
  if isfield(variantStruct,'sensorType')
    m = enumeration('SensorsEnum');
    SimIn.sensorType = m(variantStruct.sensorType);
  else
    SimIn.sensorType = selectSensorType;
  end

  Out = SimIn;
  vs = variantStruct;
end
