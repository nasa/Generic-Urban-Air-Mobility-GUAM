function fmType = selectForceMomentType()

% select vehicle type

[fm_m,fm_s]=enumeration('ForceMomentEnum');
fm_len=length(fm_m);

fprintf('\n---------------------------------------\n')
fprintf('Force/Moment Type:\n')
for i=1:fm_len
  fprintf(' (%d) %s\n', int8(fm_m(i)), fm_s{i}); 
end
selFm = input('Select aero/propulsion force/moment model: ');

fmType = fm_m(selFm);

fprintf('\n---------------------------------------');
fprintf('\n**  Force/Moment Type: %s **', fmType);
fprintf('\n---------------------------------------\n');
