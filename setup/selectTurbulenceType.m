function turbType = selectTurbulenceType()

% select Turbulence type

[turb_m,turb_s]=enumeration('TurbulenceEnum');
turb_len=length(turb_m);

fprintf('\n---------------------------------------\n')
fprintf('Turbulence Type:\n')
for i=1:turb_len
  fprintf(' (%d) %s\n', int8(turb_m(i)), turb_s{i}); 
end
selTurb = input('Select turbulence type: ');

turbType = turb_m(selTurb);

fprintf('\n---------------------------------------')
fprintf('\n**  Turbulence: %s **', turbType);
fprintf('\n---------------------------------------\n');
