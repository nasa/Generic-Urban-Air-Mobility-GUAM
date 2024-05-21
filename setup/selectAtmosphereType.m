function atmosType = selectAtmosphereType()

% select Atmosphere type

[atmos_m,atmos_s]=enumeration('AtmosphereEnum');
atmos_len=length(atmos_m);

fprintf('\n---------------------------------------\n')
fprintf('Atmosphere Type:\n')
for i=1:atmos_len
  fprintf(' (%d) %s\n', int8(atmos_m(i)), atmos_s{i}); 
end
selAtmos = input('Select atmosphere type: ');

atmosType = atmos_m(selAtmos);

fprintf('\n---------------------------------------')
fprintf('\n**  Atmosphere: %s **', atmosType);
fprintf('\n---------------------------------------\n');
