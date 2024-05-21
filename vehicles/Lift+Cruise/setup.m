function Out = setup(SimIn, target, releaseMode)
  arguments
    SimIn struct
    target struct = [];
    releaseMode logical = false;
  end
  
  % setup the number of engines and control surfaces
  SimIn.numEngines = 9;
  SimIn.numSurfaces = 5;  % e.g. left flaperon, right flaperon, left elevator, right elevator, rudder

  % build scripts for the different model types
  % Note: inertial properties are currently different for the two variants
  switch SimIn.fmType
    case 'SFunction'
      % Build model of the lift+cruise aircraft
      if releaseMode == false
        disp('Standard Mode');
        SimIn.Model = build_Lift_plus_Cruise;
      else
        disp('Release Mode');
        % Modification: no changes with release mode
        SimIn.Model = build_Lift_plus_Cruise;
      end
      %   CModel = common model parameters
      SimIn.CModel = LpC_model_parameters(SimIn); 
      % the following properties are needed to calculate propulsion gyroscopic effects
      % CModel.Ip;
      % CModel.Prop_rot_axis_e;
      % CModel.prop_spin;

      % remove parameters already in "SimIn.Model"
      SimIn.CModel=rmfield(SimIn.CModel,{'mass','I','cm_b'});

    case 'Polynomial'
      % Build model script for the polynomial model
      %   Model = polynomial model parameters
      %   CModel = common model parameters
      [SimIn.Model] = LpC_model_parameters(SimIn);

      % Lift+Cruise polynomial model version
      pmodel_option='v2.1-MOF';

      % path for the available Lift+Cruise aerodynamic models
      pmodel_v2p1_mof_path = 'AeroProp/Polynomial/LpC_Polynomial_Models_v2p1/MOF_k0p2';

      % remove aerodynamic models not selected from the search path
      lastwarn('')% clear any previous warning messages
      switch pmodel_option
        case 'v2.1-MOF'
          addpath([SimIn.vehiclepath,pmodel_v2p1_mof_path])
        otherwise
          error('invalid model')
      end

      % Check if a warning message was returned from the "rmpath" or
      % "addpath" function. If a warning was returned, a directory
      % required to be removed from/added to the search path was not
      % successfully removed/added. This check is important to ensure
      % that the correct aerodynamic model is being called in the
      % simulation.
      [warn_msg]=lastwarn;
      if ~isempty(warn_msg)
        error('A model directory path was not successfully removed/added on the search path.')
      end

      % print the aerodynamic model being used.
      fprintf('Lift+Cruise polynomial aerodynamic model: %s\n',pmodel_option)

    otherwise
      error('fmType type not defined')
  end

  % Set environmental conditions, e.g. geodesy, atmosphere, winds, turbulence
  SimIn.Environment = setupEnvironment(SimIn);

  SimIn = setupVehicle(SimIn, target);

  Out = SimIn;

end