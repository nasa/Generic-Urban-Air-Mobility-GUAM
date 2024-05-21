% define refinput variant conditions
refin_m = enumeration('RefInputEnum');
refin_len = length(refin_m);
for i=1:refin_len
  cond_expr = sprintf('SimIn.refInputType == RefInputEnum.%s', refin_m(i));
  switch refin_m(i)
    case RefInputEnum.FOUR_RAMP
      GVS_REFINPUT_TYPE_4RAMP = Simulink.Variant(cond_expr);
    case RefInputEnum.ONE_RAMP
      GVS_REFINPUT_TYPE_1RAMP = Simulink.Variant(cond_expr);
    case RefInputEnum.TIMESERIES
      GVS_REFINPUT_TYPE_TIMESERIES = Simulink.Variant(cond_expr);
    case RefInputEnum.BEZIER
      GVS_REFINPUT_TYPE_BEZIER = Simulink.Variant(cond_expr);
    case RefInputEnum.DEFAULT
      GVS_REFINPUT_TYPE_DEFAULT = Simulink.Variant(cond_expr);
    otherwise
  end
end

clear refin_m refin_len i cond_expr;