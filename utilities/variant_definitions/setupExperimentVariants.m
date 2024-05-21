% define actuator variant conditions
exp_m = enumeration('ExperimentEnum');
exp_len = length(exp_m);
for i=1:exp_len
  cond_expr = sprintf('SimIn.expType == ExperimentEnum.%s', exp_m(i));
  switch exp_m(i)
      case ExperimentEnum.DEFAULT
          GVS_EXP_TYPE_DEFAULT = Simulink.Variant(cond_expr);
      case ExperimentEnum.ATMX_TURB
          GVS_EXP_TYPE_ATMXTURB = Simulink.Variant(cond_expr);
      case ExperimentEnum.BENCHMARK
          GVS_EXP_TYPE_BENCHMARK = Simulink.Variant(cond_expr);
      case ExperimentEnum.FULL_SIMOUT
          GVS_EXP_TYPE_FULL_SIMOUT = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear exp_m exp_len i cond_expr;