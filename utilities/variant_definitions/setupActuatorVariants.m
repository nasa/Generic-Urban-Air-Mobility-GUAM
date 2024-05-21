% define actuator variant conditions
act_m = enumeration('ActuatorEnum');
act_len = length(act_m);
for i=1:act_len
  cond_expr = sprintf('SimIn.actType == ActuatorEnum.%s', act_m(i));
  switch act_m(i)
      case ActuatorEnum.None
          GVS_ACT_TYPE_NONE = Simulink.Variant(cond_expr);
      case ActuatorEnum.FirstOrder
          GVS_ACT_TYPE_FIRSTORDER = Simulink.Variant(cond_expr);
      case ActuatorEnum.SecondOrder
          GVS_ACT_TYPE_SECONDORDER = Simulink.Variant(cond_expr);
      case ActuatorEnum.FirstOrderFailSurf
          GVS_ACT_TYPE_FIRSTORDERFAILSURF = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear act_m act_len i cond_expr;