function ctrlType = selectControllerType()

% select controller type

[ctrl_m,ctrl_s]=enumeration('CtrlEnum');
ctrl_len = length(ctrl_m);

fprintf('\n---------------------------------------\n')
fprintf('Controller Type:\n')
for i=1:ctrl_len
  fprintf(' (%d) %s\n', int8(ctrl_m(i)), ctrl_s{i}); 
end
selCtrl = input('Select controller: ');

ctrlType = ctrl_m(selCtrl);

fprintf('\n---------------------------------------');
fprintf('\n**  Controller Type: %s **', ctrlType);
fprintf('\n---------------------------------------\n');
