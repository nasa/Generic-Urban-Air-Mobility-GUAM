function expType = selectExperimentType()

% select Experiment type

[exp_m,exp_s]=enumeration('ExperimentEnum');
exp_len=length(exp_m);

fprintf('\n---------------------------------------\n')
fprintf('Experiment Type:\n')
for i=1:exp_len
  fprintf(' (%d) %s\n', int8(exp_m(i)), exp_s{i}); 
end
selExp = input('Select experiment type: ');

expType = exp_m(selExp);

fprintf('\n---------------------------------------')
fprintf('\n**  Experiment: %s **', expType);
fprintf('\n---------------------------------------\n');
