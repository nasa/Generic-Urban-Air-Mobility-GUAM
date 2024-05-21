% define atmosphere variant conditions
atmos_m = enumeration('AtmosphereEnum');
atmos_len = length(atmos_m);
for i=1:atmos_len
  cond_expr = sprintf('SimIn.atmosType == AtmosphereEnum.%s', atmos_m(i));
  switch atmos_m(i)
      case AtmosphereEnum.US_STD_ATMOS_76
          GVS_ATMOS_TYPE_US76 = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear atmos_m atmos_len i cond_expr;