% close any previously loaded vehicle-specific model
close_system('GUAM',0);

% setup default base paths
setupPath;

% setup the structure containing enumeration values used to set types in
% SimIn to determine variant selections (used by setupTypes.m)
setupVariantStruct;

% assign type choice selections into SimIn structure
SimIn = setupTypes(SimIn, userStruct.variants);

% set switch settings in SimIn structure
setupSwitches;

% If Bezier and traj file specified, set SimIn accordingly
if SimIn.refInputType == RefInputEnum.BEZIER
    if (exist('userStruct','var') && isfield(userStruct,'trajFile'))
      SimIn.trajFile = userStruct.trajFile;
    end
end

% set default units
SimIn.Units=setUnits('ft','slug');
  
% Set reference inputs
if SimIn.Switches.RefTrajOn
    if SimIn.refInputType == RefInputEnum.TIMESERIES  
        % Timeseries trajectory option
        if ~(exist('target','var') && isfield(target,'RefInput'))
            % Setup default trajectory if one is not provided
            time = 0:40;
            target.RefInput.Vel_bIc_des    = timeseries(zeros(length(time),3),time);
            target.RefInput.vel_des        = timeseries(zeros(length(time),3),time);
            target.RefInput.pos_des        = timeseries(zeros(length(time),3),time);
            target.RefInput.chi_des        = timeseries(zeros(length(time),1),time);
            target.RefInput.chi_dot_des    = timeseries(zeros(length(time),1),time);
        end
        Qfields = fieldnames(target.RefInput);
        for Qn = 1:length(Qfields)
            if ~ismatrix(target.RefInput.(Qfields{Qn}).Data)
                fprintf('Reference Input Dimension > 2. Check variable (%s) and time orientation.\n',Qfields{Qn});
            end
        end
    elseif SimIn.refInputType == RefInputEnum.BEZIER 
        % Bezier trajectory option
        % NOTE: Trajectory priorites: 1) use userStruct.trafFile if provided 
        % 2) use target.RefInput if given or 3) Create default waypoints (hover)
        if (exist('userStruct','var') && isfield(userStruct, 'trajFile') && ~isempty(userStruct.trajFile)) && ...
            (exist('target','var') && isfield(target,'RefInput'))
            error(sprintf('Both a userStruct.trajFile and target.RefInput were provided to specify Bezier trajectory.\nDelete one of these Bezier trajectory specifications and rerun simSetup!!!\n'));
        end
            
        if (exist('userStruct','var') && isfield(userStruct, 'trajFile') && ~isempty(userStruct.trajFile))
            % If no RefInput but filename, then read file to create RefInput
            if ~exist(userStruct.trajFile,'file') && ~exist([userStruct.trajFile '.mat'],'file')
                % If Bezier trajFile doesn't exist, then error out and exit
                error('The Bezier trajectory file (specified in userStruct.trajFile): %s\n Does not exist!  Can''t load the desired Bezier trajectory! (be sure to include .mat extension\n', userStruct.trajFile);
            end
            % Load the data file and create the target.RefInput structure
            % Load in waypoints and time waypoints 
            % NOTE: .mat data file format: pwcurve.waypoints, pwcurve.time_wpts
            % NOTE: waypoints are expressed in NED frame
            temp = load(userStruct.trajFile);
            target.RefInput.Bezier.waypoints =temp.pwcurve.waypoints;
            target.RefInput.Bezier.time_wpts = temp.pwcurve.time_wpts;

            % Set up the initial conditions
            [pos_i, vel_i, acc_i, chi, chid] = evalSegments(target.RefInput.Bezier.waypoints{1}, target.RefInput.Bezier.waypoints{2}, target.RefInput.Bezier.waypoints{3},...
                target.RefInput.Bezier.time_wpts{1},target.RefInput.Bezier.time_wpts{2},target.RefInput.Bezier.time_wpts{3}, 0);
            % vel_i in NED frame so need to convert to control frame
            Q_i2c = [cos(chi/2), 0*sin(chi/2), 0*sin(chi/2), sin(chi/2)]';
            target.RefInput.Vel_bIc_des    =  Qtrans(Q_i2c,vel_i);
            target.RefInput.pos_des        = pos_i;
            target.RefInput.chi_des        = chi;
            target.RefInput.chi_dot_des    = chid;
            % set the traj start and stop times
            % NOTE: if time_wpts is cell, then start and stop times of each
            % cell array are assummed to start and stop at same times
            if iscell(target.RefInput.Bezier.time_wpts)
                target.RefInput.trajectory.refTime = [target.RefInput.Bezier.time_wpts{1}(1) target.RefInput.Bezier.time_wpts{1}(end)];
            else
                target.RefInput.trajectory.refTime = [target.RefInput.Bezier.time_wpts(1) target.RefInput.Bezier.time_wpts(end)];
            end
            clear temp
        elseif (exist('target','var') && isfield(target,'RefInput'))
            % NOTE: No action needed if target.RefInput is provided
        else
            % Need to set up default trajectory (hover at zero velocity)
            % Store the initial trajectory
            target.RefInput.Bezier.waypoints = {zeros(2,3), zeros(2,3), zeros(2,3)};
            target.RefInput.Bezier.time_wpts = [0 10];
            % Set desired Initial condition vars
            target.RefInput.Vel_bIc_des    = zeros(3,1);
            target.RefInput.pos_des        = zeros(3,1);
            target.RefInput.chi_des        = 0;
            target.RefInput.chi_dot_des    = 0;
            target.RefInput.trajectory.refTime = [0 10]; % Ten second traj
        end
    end
else
    if exist('target','var') % target variable specified
        % Check user at least supplied: tas, gndtrack and stopTime
        if ~(isfield(target,'tas') && isfield(target,'gndtrack') && isfield(target,'stopTime'))
            error('For user specified target structure: tas, gndtrack and stopTime must be provided for non-TIMESERIES refInput Type');
        end
    else
        % Specify default conditions if user didn't specify neither
        % Timeseries nor minimal target structure
        target = struct('tas', 110, 'gndtrack', 0, 'stopTime', 10);
    end

end

% once all choices are selected, run setup
SimIn = setup(SimIn, target);  % standard simulation mode
%SimIn = setup(SimIn, target, true);  % release simulation mode

% Setup Parameters
setupParameters(SimIn);

% create variant control objects
setupVariants;

% create buses
setupBuses;

% Determmine which user prescribed output file to utilize..
if ~isfield(userStruct,'outputFname') || isempty(userStruct.outputFname)
    % Utilize the default fname
    [res] = copyfile(sprintf('./%s/setup/defaultSimOut.m',vehDir), sprintf('./%s/setup/mySimOutFunc.m',vehDir));
    clear 'BUS_USER_*' vehDir
else % Determine if the user defined name is valid
    tFname = split(deblank(userStruct.outputFname),'.'); % Remove .m file extension if provided
    [res] = copyfile(sprintf('%s/setup/User_SimOut/%s.m',vehDir,tFname{1}), sprintf('%s/setup/mySimOutFunc.m',vehDir));
    % Delete any buses that start with 'BUSOUT_' to ensure that the correct user specified buses are used..
    clear 'BUS_USER_*' vehDir
end

% Build the userdefined SimOut bus structure and update GUAM
Build_User_SimOut_BUS('mySimOutFunc', 'BUS_USER_', 'BUS_SIM_OUT')

% Clear target after setup (so it won't be accidentally used next run)
clear target