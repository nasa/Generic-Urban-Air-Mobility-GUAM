function eomType = selectEOMType()

% select vehicle type

[eom_m,eom_s]=enumeration('EOMEnum');
eom_len=length(eom_m);

fprintf('\n\n');
fprintf('\n---------------------------------------\n')
fprintf('EOM Type:\n')
for i=1:eom_len
  fprintf(' (%d) %s\n', int8(eom_m(i)), eom_s{i}); 
end
selEOM = input('Select EOM: ');

eomType = eom_m(selEOM);

fprintf('\n---------------------------------------');
fprintf('\n**  EOM Type: %s **', eomType);
fprintf('\n---------------------------------------\n');
