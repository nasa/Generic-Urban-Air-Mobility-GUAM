function propType = selectPropulsionType(ctrlType)

% select Propulsion type

[prop_m,prop_s]=enumeration('PropulsionEnum');
prop_len=length(prop_m);

% check if ctrl type is TRIM, if so, set propulsion type to None
if ctrlType == CtrlEnum.TRIM
  propType = PropulsionEnum.None;
else
  fprintf('\n---------------------------------------\n')
  fprintf('Propulsion Type:\n')
  for i=1:prop_len
    fprintf(' (%d) %s\n', int8(prop_m(i)), prop_s{i}); 
  end
  selProp = input('Select propulsion type: ');

  propType = prop_m(selProp);
end

fprintf('\n---------------------------------------')
fprintf('\n**  Propulsion: %s **', propType);
fprintf('\n---------------------------------------\n');
