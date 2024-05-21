function vehicleType = selectVehicleType()

% select vehicle type

[veh_m,veh_s]=enumeration('VehicleEnum');
veh_len=length(veh_m);

fprintf('\n\n');
fprintf('\n---------------------------------------\n')
fprintf('Vehicle Type:\n')
for i=1:veh_len
  fprintf(' (%d) %s\n', int8(veh_m(i)), veh_s{i}); 
end
selVeh = input('Select vehicle: ');

vehicleType = veh_m(selVeh);

fprintf('\n---------------------------------------');
fprintf('\n**  Vehicle Type: %s **', vehicleType);
fprintf('\n---------------------------------------\n');
