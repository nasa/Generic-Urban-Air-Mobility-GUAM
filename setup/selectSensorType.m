function sensorType = selectSensorType()

% select Sensor type

[sen_m,sen_s]=enumeration('SensorsEnum');
sen_len=length(sen_m);

fprintf('\n---------------------------------------\n')
fprintf('Sensor Type:\n')
for i=1:sen_len
  fprintf(' (%d) %s\n', int8(sen_m(i)), sen_s{i}); 
end
selSen = input('Select sensor type: ');

sensorType = sen_m(selSen);

fprintf('\n---------------------------------------')
fprintf('\n**  Sensor: %s **', sensorType);
fprintf('\n---------------------------------------\n');
