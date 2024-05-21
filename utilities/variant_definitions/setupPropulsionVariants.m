% define propulsion variant conditions
prop_m = enumeration('PropulsionEnum');
prop_len = length(prop_m);
for i=1:prop_len
  cond_expr = sprintf('SimIn.propType == PropulsionEnum.%s', prop_m(i));
  switch prop_m(i)
      case PropulsionEnum.None
          GVS_PROP_TYPE_NONE = Simulink.Variant(cond_expr);
      case PropulsionEnum.FirstOrder
          GVS_PROP_TYPE_FIRSTORDER = Simulink.Variant(cond_expr);
      case PropulsionEnum.SecondOrder
          GVS_PROP_TYPE_SECONDORDER = Simulink.Variant(cond_expr);
      case PropulsionEnum.FirstOrderFailProp
          GVS_PROP_TYPE_FIRSTORDERFAILPROP = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear prop_m prop_len i cond_expr;