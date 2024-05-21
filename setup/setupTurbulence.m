function Out = setupTurbulence(SimIn)

  % Placeholder for turbulence;
  Out.Vel_tBb     = [0;0;0];
  Out.VelDtB_tBb  = [0;0;0];
  Out.Omeg_TBb    = [0;0;0];
  Out.OmegDtB_TBb = [0;0;0];

  % Inputs for the Turbulence Model-
  % The values are used in the Mask interface
  switch SimIn.fmType
      case 'SFunction'
          Out.Vehicle_span_m = SimIn.Model.WingProp.Wing.b/SimIn.Units.m;  % wingspan in meters
      case 'Polynomial'
          Out.Vehicle_span_m = SimIn.Model.b/SimIn.Units.m;  % wingspan in meters
      otherwise
          error('SimIn.fmType is not properly defined')
  end
  Out.RandomSeeds          = [ 23341, 23342, 23343, 23344];  % [ u_turb, v_turb, w_turb, p_gust]  (q & r driven by v and w)  


  Out.WindAt5kft           = 00;    % m/s
  Out.WindDirectionAt5kft  = 0 ;    % deg CW from North
  Out.dT                   = 0.005; %  must be a fixed step and match simulation time step 
  %Out.IntensityLevel       = 1;    %  Turbulence Intensity 1=Light, 2= Moderate, 3 =Severe  <-gain on RMS

  Out.MeanWindGain         = 33;   %  gain 0-100 % factor applied to MEAN WIND 
                                   %  ( MeanWindGain scales the Mean Wind down ) 
                                   %  ( when the value is <100                 ) 
                                  
end