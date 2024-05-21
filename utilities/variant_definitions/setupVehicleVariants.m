% define vehicle variant conditions
veh_m = enumeration('VehicleEnum');
veh_len = length(veh_m);
for i=1:veh_len
  cond_expr = sprintf('SimIn.vehicleType == VehicleEnum.%s', veh_m(i));
  switch veh_m(i)
    case VehicleEnum.LiftPlusCruise
      GVS_VEH_TYPE_LC = Simulink.Variant(cond_expr);
    case VehicleEnum.Quad6
      GVS_VEH_TYPE_QUAD6 = Simulink.Variant(cond_expr);
    case VehicleEnum.GenTiltRotor
      GVS_VEH_TYPE_GTR = Simulink.Variant(cond_expr);
    case VehicleEnum.GenTiltWing
      GVS_VEH_TYPE_GTW = Simulink.Variant(cond_expr);
    case VehicleEnum.GL10
      GVS_VEH_TYPE_GL10 = Simulink.Variant(cond_expr);
    case VehicleEnum.LA8
      GVS_VEH_TYPE_LA8 = Simulink.Variant(cond_expr);
    otherwise
  end
end

clear veh_m veh_len i cond_expr;