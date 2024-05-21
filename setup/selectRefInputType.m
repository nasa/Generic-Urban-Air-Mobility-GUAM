function refInputType = selectRefInputType()

% select ref input type

[refin_m,refin_s]=enumeration('RefInputEnum');
refin_len = length(refin_m);

fprintf('\n---------------------------------------\n')
fprintf('RefInput Type:\n')
for i=1:refin_len
  fprintf(' (%d) %s\n', int8(refin_m(i)), refin_s{i}); 
end
selRefIn = input('Select ref input: ');

refInputType = refin_m(selRefIn);

fprintf('\n---------------------------------------');
fprintf('\n**  RefInput Type: %s **', refInputType);
fprintf('\n---------------------------------------\n');
