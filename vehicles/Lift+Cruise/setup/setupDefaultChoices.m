function [Out] = setupDefaultChoices(variantStruct, bExist)
  arguments
    variantStruct struct
    bExist logical
  end

  % default Lift+Cruise selections
  if ~bExist || (bExist && ~isfield(variantStruct,'ctrlType'))
    variantStruct.ctrlType = CtrlEnum.BASELINE;
  elseif (bExist && (variantStruct.ctrlType == -1))
    disp('User requested selection menu for controller type');
    variantStruct = rmfield(variantStruct,'ctrlType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'refInputType'))
    variantStruct.refInputType = RefInputEnum.DEFAULT;
  elseif (bExist && (variantStruct.refInputType == -1))
    disp('User requested selection menu for ref input type');
    variantStruct = rmfield(variantStruct,'refInputType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'actType'))
    variantStruct.actType = ActuatorEnum.FirstOrder;
  elseif (bExist && (variantStruct.actType == -1))
    disp('User requested selection menu for surface actuator type');
    variantStruct = rmfield(variantStruct,'actType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'propType'))
    variantStruct.propType = PropulsionEnum.None;
  elseif (bExist && (variantStruct.propType == -1))
    disp('User requested selection menu for prop actuator type');
    variantStruct = rmfield(variantStruct,'propType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'fmType'))
    %variantStruct.fmType = ForceMomentEnum.SFunction;
    variantStruct.fmType = ForceMomentEnum.Polynomial;
  elseif (bExist && (variantStruct.fmType == -1))
    disp('User requested selection menu for force/moment type');
    variantStruct = rmfield(variantStruct,'fmType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'eomType'))
    % variantStruct.eomType = EOMEnum.STARS;
    variantStruct.eomType = EOMEnum.Simple;
  elseif (bExist && (variantStruct.eomType == -1))
    disp('User requested selection menu for EOM type');
    variantStruct = rmfield(variantStruct,'eomType');
  end
  if ~bExist || (bExist && ~isfield(variantStruct,'sensorType'))
    variantStruct.sensorType = SensorsEnum.None;
  elseif (bExist && (variantStruct.sensorType == -1))
    disp('User requested selection menu for sensor type');
    variantStruct = rmfield(variantStruct,'sensorType');
  end

  Out = variantStruct;
  
end

