% define EOM variant conditions
eom_m = enumeration('EOMEnum');
eom_len = length(eom_m);
for i=1:eom_len
  cond_expr = sprintf('SimIn.eomType == EOMEnum.%s', eom_m(i));
  switch eom_m(i)
      case EOMEnum.STARS
          GVS_EOM_TYPE_LIMITED = Simulink.Variant(cond_expr);
      case EOMEnum.Simple
          GVS_EOM_TYPE_SIMPLE = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear eom_m eom_len i cond_expr;