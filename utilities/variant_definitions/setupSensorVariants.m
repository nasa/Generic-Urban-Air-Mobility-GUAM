% define sensor variant conditions
sen_m = enumeration('SensorsEnum');
sen_len = length(sen_m);
for i=1:sen_len
  cond_expr = sprintf('SimIn.sensorType == SensorsEnum.%s', sen_m(i));
  switch sen_m(i)
      case SensorsEnum.None
          GVS_SENSOR_TYPE_NONE = Simulink.Variant(cond_expr);
      case SensorsEnum.ZOH
          GVS_SENSOR_TYPE_ZOH = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear sen_m sen_len i cond_expr;