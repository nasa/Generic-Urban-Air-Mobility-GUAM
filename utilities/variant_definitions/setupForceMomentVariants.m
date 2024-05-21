% define force/moment variant conditions
fm_m = enumeration('ForceMomentEnum');
fm_len = length(fm_m);
for i=1:fm_len
  cond_expr = sprintf('SimIn.fmType == ForceMomentEnum.%s', fm_m(i));
  switch fm_m(i)
      case ForceMomentEnum.SFunction
          GVS_FM_TYPE_SFUNCTION = Simulink.Variant(cond_expr);
      case ForceMomentEnum.Polynomial
          GVS_FM_TYPE_POLYNOMIAL = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear fm_m fm_len i cond_expr;