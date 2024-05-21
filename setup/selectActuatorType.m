function actType = selectActuatorType(ctrlType)

% select Actuator type

[act_m,act_s]=enumeration('ActuatorEnum');
act_len=length(act_m);

%{
% check if ctrl type is TRIM, if so, set actuator type to None
if ctrlType == CtrlEnum.TRIM
  actType = ActuatorEnum.None;
else
  fprintf('\n---------------------------------------\n')
  fprintf('Actuator Type:\n')
  for i=2:act_len
    fprintf(' (%d) %s\n', int8(act_m(i))-1, act_s{i}); 
  end
  selAct = input('Select actuator type: ');

  actType = act_m(selAct+1);
end
%}
  fprintf('\n---------------------------------------\n')
  fprintf('Actuator Type:\n')
  for i=1:act_len
    fprintf(' (%d) %s\n', int8(act_m(i)), act_s{i}); 
  end
  selAct = input('Select actuator type: ');

  actType = act_m(selAct);

fprintf('\n---------------------------------------')
fprintf('\n**  Actuator: %s **', actType);
fprintf('\n---------------------------------------\n');
