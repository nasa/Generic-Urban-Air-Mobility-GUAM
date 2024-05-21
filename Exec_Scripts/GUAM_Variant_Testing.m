% This script runs thru all the variant options to make sure they are
% viable

%      vehicleType: LiftPlusCruise
%          expType: DEFAULT
%        atmosType: US_STD_ATMOS_76
%         turbType: None
%         ctrlType: BASELINE
%     refInputType: DEFAULT
%          actType: FirstOrder
%         propType: None
%           fmType: SFunction
%          eomType: Simple
%       sensorType: None

model = 'GUAM'; % Define model name
for loop = [1:7 9:10] %1:9
    clear userStruct
    setupVariantStruct; % Initialize the variant structure
    switch loop
        case 1 % Actuators
            for eloop = 1:4
                userStruct.variants.actType = eloop;
                [fail_flag] = Build_Test(userStruct, ActuatorEnum(eloop), model);
            end
        case 2 % Atmosphere
            for eloop = 1
                userStruct.variants.atmosType = eloop;
                [fail_flag] = Build_Test(userStruct, AtmosphereEnum(eloop), model);
            end
        case 3 % EOM
            for eloop = 1:2
                userStruct.variants.eomType = eloop;
                [fail_flag] = Build_Test(userStruct, EOMEnum(eloop), model);
            end
        case 4 % Experiment
            for eloop = 1:4
                userStruct.variants.expType = eloop;
                [fail_flag] = Build_Test(userStruct, ExperimentEnum(eloop), model);
            end
        case 5 % Force and Moment
            for eloop = 1:2
                userStruct.variants.fmType = eloop;
                [fail_flag] = Build_Test(userStruct, ForceMomentEnum(eloop), model);
            end
        case 6 % Propulsors
            for eloop = 1:4
                userStruct.variants.propType = eloop;
                [fail_flag] = Build_Test(userStruct, PropulsionEnum(eloop), model);
            end
        case 7 % Sensors
            for eloop = 1:2
                userStruct.variants.sensorType = eloop;
                [fail_flag] = Build_Test(userStruct, SensorsEnum(eloop), model);
            end
        case 8 % System Type (linearization)
            for eloop = 1:2
                userStruct.variants. = eloop;
                [fail_flag] = Build_Test(userStruct, SensorsEnum(eloop), model);
            end
        case 9 % Turbulence
            for eloop = 1:2
                userStruct.variants.turbType = eloop;
                [fail_flag] = Build_Test(userStruct, TurbulenceEnum(eloop), model);
            end
        case 10 % Vehicle Type
            for eloop = 1:7
                userStruct.variants.vehicleType = eloop;
                [fail_flag] = Build_Test(userStruct, VehicleEnum(eloop), model);
            end
    end % End switch loop
end % End for loop = 1:9
function [fail_flag] = Build_Test(userStruct, enum, model_name)
    % Try to build the sim with the variant selected see if it fails
    fail_flag = 0;
    try
        simSetup;
        open(model_name);
        set_param(gcs,'SimulationCommand','Update');
        disp('');
        % rtwbuild(model_name);
    catch
        fail_flag = 1;
        fprintf('Sim fails to build correctly for the following enumeration:\n');
        enum
        disp('');
    end
end