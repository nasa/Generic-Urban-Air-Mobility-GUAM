function mode = selectFlightMode()

% select Flight Mode

[mode_m,mode_s]=enumeration('FlightModeEnum');
mode_len=length(mode_m);

fprintf('\n---------------------------------------\n')
fprintf('Flight Mode:\n')
for i=1:mode_len
  fprintf(' (%d) %s\n', int8(mode_m(i)), mode_s{i}); 
end
selMode = input('Select flight mode: ');

mode = mode_m(selMode);

fprintf('\n---------------------------------------');
fprintf('\n**  Mode: %s **', mode);
fprintf('\n---------------------------------------\n');
