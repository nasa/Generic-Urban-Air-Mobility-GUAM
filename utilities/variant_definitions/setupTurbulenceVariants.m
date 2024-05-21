% define turbulence variant conditions
turb_m = enumeration('TurbulenceEnum');
turb_len = length(turb_m);
for i=1:turb_len
  cond_expr = sprintf('SimIn.turbType == TurbulenceEnum.%s', turb_m(i));
  switch turb_m(i)
      case TurbulenceEnum.None
          GVS_TURB_TYPE_NONE = Simulink.Variant(cond_expr);
      case TurbulenceEnum.Enabled
          GVS_TURB_TYPE_ENABLED = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear turb_m turb_len i cond_expr;