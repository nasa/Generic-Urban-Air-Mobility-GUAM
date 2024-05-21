% *************************************************************************
% ctrl_scheduler_GUAM:
%
% This script is used to design and output a series of gain scheduled unified
% controllers (separate longitudinal and lateral controllers).  
% % It requires trim file(s), which are concatenated into monotonic
% increasing arrays of control frame body velocities (UH and WH) and potentially
% others arrays (e.g., R => turn radius, Acc => acceleration).  The controllers 
% designed are based on Dr. Jacob Cook's unified controller (i.e., same control 
% variables across the hover, transition, and cruise flight phases of transition 
% vehicles).  See Ref: "Examination of Unified Control Approaches Incorporating
% Generalized Control Allocation", AIAA 2021-0999
%
% Written by Jacob Cook, NASA Langley Research Center.
% Current contact michael.j.acheson@nasa.gov, (757)-864-9457
% Dynamics Systems and Control Branch (DSCB D-316)
%
% INPUTS:
%   Trim Files (delineated in m-script Concatenate_Trim_Files.m)
% OUTPUTS:
%   Trim/Controller output .mat file for use in Generalized Urban Air Mobility
%   (GUAM) or Generalized Vehicle Sim (GVS)
% OTHER UTILIZED FUNCTIONS:
%   Concatenate_Trim_Files.m: % Concatenates trim files 
%   ctrl_lon.m  % Designs unified longitudinal controller (see Ref.)
%   ctrl_lat.m  % Designs unified lateral controller (see Ref.)
% *************************************************************************

% VERSION HISTORY
% 7.16.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS
% 
% 7.20.2023, Michael J. Acheson (NASA LaRC D-316): Updated version to
% include modified longitudinal and lateral control design m-files, changes
% to output controller based on desired control effectors (per phase of
% flight), inclusion of full polynomial aero/propulsive L+C database
% functionality.
% 

global POLY % Define global variable to switch between aero/propulsive L+C databases
POLY = 1; % 1=> use polynomial aero/propulsive (A/P) database, 0=> use strip theory s-function A/P database

% Specify necessary constants (e.g., gravity and air density)
rho  = 0.0023769; % slugs/ft^3
grav = 32.17405; % ft/sec^2
ft2kts = 1/(1852.0/0.3048/3600); % Obtained from setUnits... Old = 0.592484; % Conversion from ft/sec to kts

% *************************************************************************
fprintf('Control Scheduler \n');

% *********************** USER INPUT SECTION ******************************
fprintf('Concatenating Trim Files');
% Provide the input trim path and filenames (Trim_Ver1p0)
% fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver1p0';
% trim_fnames = {'Trim_Case_-6.1_RInf_WH-11.7_XEQ.mat', 'Trim_Case_6_RInf_WH0_XEQ.mat', 'Trim_Case_6.1_RInf_WH11.7_XEQ.mat'}; % Trim_Ver1p0
% out_trim_fname  = 'Trim_poly_XEQ_Concat.mat';% Trim_Ver1p0
% out_cntr_fname = './trim_table_Poly_Concat.mat'; % Output file for Trim_Ver1p0 (need to change file read in SetupControl.m)

% % Provide the input trim path and filenames (Trim_Ver2p0)
% fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver2p0';
% trim_fnames = {'Trim_Case_-6.2_RInf_WH-11.7_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat', 'Trim_Case_6.2_RInf_WH11.7_XEQ.mat'}; % Trim_Ver2p0
% out_trim_fname  = 'Trim_poly_XEQ_ConcatV2p0.mat'; % Trim_Ver2p0
% out_cntr_fname = './trim_table_Poly_ConcatVer2p0.mat'; % Output file for Trim_Ver2p0 (need to change file read in SetupControl.m)

% Provide the input trim path and filenames (Trim_Ver2p0)
%fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver3p0';
fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver4p0';

%trim_fnames = {'Trim_Case_-6.3_RInf_WH-11.7_XEQ.mat', 'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
%trim_fnames = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
% trim_fnames  = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat','Trim_Case_6.3_RInf_WH0_XEQ.mat','Trim_Case_6.3_RInf_WH11.7_XEQ.mat'}; % Trim_Ver3p0
% trim_fnames  = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat','Trim_Case_6.3_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
 trim_fnames  = {'Trim_Case_-6.5_RInf_WH-7.5_XEQ.mat','Trim_Case_6.4_RInf_WH0_XEQ.mat','Trim_Case_6.4_RInf_WH11.7_XEQ.mat'}; % Trim_Ver3p0
% out_trim_fname  = 'Trim_poly_XEQ_ConcatV3p0.mat'; % Trim_Ver3p0
% out_cntr_fname = './trim_table_Poly_ConcatVer3p0.mat'; % Output file for Trim_Ver3p0 (need to change file read in SetupControl.m)
out_trim_fname  = 'Trim_poly_XEQ_ConcatV4p0.mat'; % Trim_Ver4p0
out_cntr_fname = './trim_table_Poly_ConcatVer4p0.mat'; % Output file for Trim_Ver4p0 (need to change file read in SetupControl.m)
% *********************** END OF USER INPUT SECTION ***********************

% Specify the filename for the trim output file that contains the combined files
out_trim_fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise';
out_cntr_fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise';
Concatenate_Trim_Files; % Concatenate trim files 

% Process the data 
load(fullfile(out_trim_fpath, out_trim_fname)); % Load resultant trim table for Trim version

% Parse the trim table
XEQ_TABLE = XEQ;
if any(any(isnan(XEQ_TABLE)))
    keyboard
end

% Determine size of gain scheduling arrays
%    UH     WH      R 
[~, N_trim, M_trim, L_trim] = size(XEQ_TABLE); 

% Initial output table
XEQ0 = zeros(21,N_trim,M_trim,L_trim);

% Build the aircraft (aero/propulsive modeling..)
if POLY  
  SimIn.numEngines = 9;
  lpc = LpC_model_parameters(SimIn);
else
  lpc = build_Lift_plus_Cruise();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control design and Gain Scheduling
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State Cost 
%         ui   wi   qi   u w q 
Qlon0 = [ 0.01 0.01 1000 0 0 0]'; % original 
%Qlon0 = [ 0.02 0.02 100 0 0 0]'; % original 


% Control acceleration cost
Rlon0 = [1 1 1]'; % original

% Control allocation weighting
%       [ omp1:omp9         dele dflap    th] 
Wlon0 = [ 1 1 1 1 1 1 1 1 1 1000 10000000 0.1]'; % Modified effector output order to match the simulation allocation
%Wlon0 = [ 1 1 1 1 1 1 1 1 1 1000 10000000 1]'; % Modified effector output order to match the simulation allocation
%Wlon0 = [ 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 1 1000 10000000 0.01]'; % Modified effector output order to match the simulation allocation

Qlon = repmat(Qlon0, [1,N_trim,M_trim,L_trim]);
Rlon = repmat(Rlon0, [1,N_trim,M_trim,L_trim]);
Wlon = repmat(Wlon0, [1,N_trim,M_trim,L_trim]);

% State Cost 
%         vi   pi   ri   v p r 
Qlat0 = [ 0.01 1000 1000 0 0 0]'; 

% Control acceleration cost
Rlat0 = [1 1 1]';

% Control allocation weighting
%       [ omp1:omp9      dela delr phi] 
Wlat0 = [1 1 1 1 1 1 1 1 1000 1000 1]'; % Modified effector output order to match the simulation allocation
%Wlat0 = [1 1 1 1 1 1 1 1 1000 1000 0.2]'; % Modified effector output order to match the simulation allocation

Qlat = repmat(Qlat0, [1,N_trim,M_trim,L_trim]);
Rlat = repmat(Rlat0, [1,N_trim,M_trim,L_trim]);
Wlat = repmat(Wlat0, [1,N_trim,M_trim,L_trim]);

% Specify system sizes
Nx_lon = 4;
Ni_lon = 3;
Nu_lon = 11;
Nr_lon = 3;
Nv_lon = 1;

Nx_lat = 4;
Ni_lat = 3;
Nu_lat = 10;
Nr_lat = 2;
Nv_lat = 1;

% *************************************************************************
%       Design Lon & Lat Controllers for all schedule intervals
% *************************************************************************

% Initialize variables
LON = [];
LAT = [];
UH  = [];
WH  = [];
R   = [];

% Locate lifting off speed
% [~,lift_off_ind] = min((XEQ_TABLE(1,:,1,1)-lifting_off_speed).^2);

for kk = 1:L_trim % R (turn radius schedule interval)
  for jj = 1:M_trim % WH (z vel in control frame gain schedule interval)
    for ii = 1:N_trim % UH (x vel in control frame schedule interval)

      % Pull out the desired trim point
      trim_pnt      = XEQ_TABLE(:,ii,jj,kk);
      if any(isnan(trim_pnt))
          keyboard
      end
      FreeVar_pnt   = FreeVar_Table(:,:, jj, kk);
      Trans_pnt     = Trans_Table(:, jj, kk);

      % use the rslqr control design to get the control gains at each point
      [lon, lon_err] = ctrl_lon(lpc, trim_pnt, rho, grav,Qlon(:,ii,jj,kk),Rlon(:,ii,jj,kk),Wlon(:,ii,jj,kk), FreeVar_pnt, Trans_pnt);
      if lon_err
        fprintf('eq. point: (%i, %i, %i)\n',ii,jj,kk);
      end

      [lat, lat_err] = ctrl_lat(lpc, trim_pnt, rho, grav,Qlat(:,ii,jj,kk),Rlat(:,ii,jj,kk),Wlat(:,ii,jj,kk), FreeVar_pnt, Trans_pnt);
      if lat_err
        fprintf('eq. point: (%i, %i, %i)\n',ii,jj,kk);
      end

      % Concatenate longitudinal and lateral controller structures
      LON = [LON lon];
      LAT = [LAT lat];

    end
  end
end

% Initialize trim/controller output file variables
UH  = reshape([XEQ_TABLE(1,:,1,1)], N_trim, 1);
WH  = reshape([XEQ_TABLE(2,1,:,1)], M_trim, 1);
R   = reshape([XEQ_TABLE(3,1,1,:)], L_trim, 1);

% Original (old table lookup format) 
XU0_interp = zeros(25, N_trim, M_trim);

% Pre allocate longitudinal directional blocks (Old format)
Ap_lon_interp = zeros(Nx_lon, Nx_lon, N_trim, M_trim);
Bp_lon_interp = zeros(Nx_lon, Nu_lon, N_trim, M_trim);
Cp_lon_interp = zeros(Nx_lon, Nx_lon, N_trim, M_trim);
Dp_lon_interp = zeros(Nx_lon, Nu_lon, N_trim, M_trim);

Ac_lon_interp = zeros(Ni_lon, Ni_lon, N_trim, M_trim);
Bc_lon_interp = zeros(Ni_lon, Nx_lon, N_trim, M_trim);
Br_lon_interp = zeros(Ni_lon, Nr_lon, N_trim, M_trim);
Cc_lon_interp = zeros(Nu_lon, Ni_lon, N_trim, M_trim);
Dc_lon_interp = zeros(Nu_lon, Nx_lon, N_trim, M_trim);
Dr_lon_interp = zeros(Nu_lon, Nr_lon, N_trim, M_trim);

Ki_lon_interp = zeros(Ni_lon, Ni_lon, N_trim, M_trim);
Kx_lon_interp = zeros(Ni_lon, Nx_lon, N_trim, M_trim);
Kv_lon_interp = zeros(Ni_lon, Nv_lon, N_trim, M_trim);

F_lon_interp  = zeros(Ni_lon, Nr_lon,N_trim, M_trim);
G_lon_interp  = zeros(Ni_lon, Nr_lon, N_trim, M_trim);
C_lon_interp  = zeros(Ni_lon, Nx_lon, N_trim, M_trim);
Cv_lon_interp = zeros(Nv_lon, Nx_lon, N_trim, M_trim);

Q_lon_interp  = zeros(2*Ni_lon, 2*Ni_lon, N_trim, M_trim);
R_lon_interp  = zeros(Ni_lon, Ni_lon, N_trim, M_trim);

W_lon_interp  = zeros((Nu_lon+Nv_lon), (Nu_lon+Nv_lon), N_trim, M_trim);
B_lon_interp  = zeros(Ni_lon, (Nu_lon+Nv_lon), N_trim, M_trim);


% Pre allocate lateral directional blocks (Old format)
Ap_lat_interp = zeros(Nx_lat, Nx_lat, N_trim, M_trim);
Bp_lat_interp = zeros(Nx_lat, Nu_lat, N_trim, M_trim);
Cp_lat_interp = zeros(Nx_lat, Nx_lat, N_trim, M_trim);
Dp_lat_interp = zeros(Nx_lat, Nu_lat, N_trim, M_trim);

Ac_lat_interp = zeros(Ni_lat, Ni_lat, N_trim, M_trim);
Bc_lat_interp = zeros(Ni_lat, Nx_lat, N_trim, M_trim);
Br_lat_interp = zeros(Ni_lat, Nr_lat, N_trim, M_trim);
Cc_lat_interp = zeros(Nu_lat, Ni_lat, N_trim, M_trim);
Dc_lat_interp = zeros(Nu_lat, Nx_lat,N_trim, M_trim);
Dr_lat_interp = zeros(Nu_lat, Nr_lat, N_trim, M_trim);

Ki_lat_interp = zeros(Ni_lat, Ni_lat, N_trim, M_trim);
Kx_lat_interp = zeros(Ni_lat, Nx_lat, N_trim, M_trim);
Kv_lat_interp = zeros(Ni_lat, Nv_lat, N_trim, M_trim);

F_lat_interp  = zeros(Ni_lat, Nr_lat, N_trim, M_trim);
G_lat_interp  = zeros(Ni_lat, Nr_lat, N_trim, M_trim);
C_lat_interp  = zeros(Ni_lat, Nx_lat, N_trim, M_trim);
Cv_lat_interp = zeros(Nv_lat, Nx_lat, N_trim, M_trim);

Q_lat_interp  = zeros(2*Ni_lat, 2*Ni_lat, N_trim, M_trim);
R_lat_interp  = zeros( Ni_lat, Ni_lat, N_trim, M_trim);

W_lat_interp  = zeros((Nu_lat+Nv_lat), (Nu_lat+Nv_lat), N_trim, M_trim);
B_lat_interp  = zeros(Ni_lat, (Nu_lat+Nv_lat), N_trim, M_trim);

% **** Store the controller outputs in the GUAM/GVS .mat format.. *********
for ii = 1:N_trim
  for jj = 1:M_trim
    for kk = 1:L_trim

        XU0_interp(:, ii,jj)    = LON(ii+(jj-1)*N_trim).XU0;

        % Longitudinal System
        Ap_lon_interp = reshape([LON.Ap],Nx_lon,Nx_lon,N_trim,M_trim);
        Bp_lon_interp = reshape([LON.Bp],Nx_lon,Nu_lon,N_trim,M_trim);
        Cp_lon_interp = reshape([LON.Cp],Nx_lon,Nx_lon,N_trim,M_trim);
        Dp_lon_interp = reshape([LON.Dp],Nx_lon,Nu_lon,N_trim,M_trim);
        
        Ac_lon_interp = reshape([LON.Ac],Ni_lon,Ni_lon,N_trim,M_trim);
        Bc_lon_interp = reshape([LON.Bc],Ni_lon,Nx_lon,N_trim,M_trim);
        Br_lon_interp = reshape([LON.Br],Ni_lon,Nr_lon,N_trim,M_trim);
        Cc_lon_interp = reshape([LON.Cc],Nu_lon,Ni_lon,N_trim,M_trim);
        Dc_lon_interp = reshape([LON.Dc],Nu_lon,Nx_lon,N_trim,M_trim);
        Dr_lon_interp = reshape([LON.Dr],Nu_lon,Nr_lon,N_trim,M_trim);
        
        Ki_lon_interp = reshape([LON.Ki],Ni_lon,Ni_lon,N_trim,M_trim);
        Kx_lon_interp = reshape([LON.Kx],Ni_lon,Nx_lon,N_trim,M_trim);
        Kv_lon_interp = reshape([LON.Kv],Ni_lon,Nv_lon,N_trim,M_trim);
        
        F_lon_interp = reshape([LON.F],Ni_lon,Nr_lon,N_trim,M_trim);
        G_lon_interp = reshape([LON.G],Ni_lon,Nr_lon,N_trim,M_trim);
        C_lon_interp = reshape([LON.C],Ni_lon,Nx_lon,N_trim,M_trim);
        Cv_lon_interp = reshape([LON.Cv],Nv_lon,Nx_lon,N_trim,M_trim);
        
        Q_lon_interp = reshape([LON.Q],2*Ni_lon, 2*Ni_lon, N_trim,M_trim);
        R_lon_interp = reshape([LON.R],Ni_lon,Ni_lon,N_trim,M_trim);
        
        W_lon_interp = reshape([LON.W],Nu_lon+Nv_lon,Nu_lon+Nv_lon,N_trim,M_trim);
        B_lon_interp = reshape([LON.B],Ni_lon,Nu_lon+Nv_lon,N_trim, M_trim);
        
        % Lateral System
        Ap_lat_interp = reshape([LAT.Ap],Nx_lat,Nx_lat,N_trim, M_trim);
        Bp_lat_interp = reshape([LAT.Bp],Nx_lat,Nu_lat,N_trim, M_trim);
        Cp_lat_interp = reshape([LAT.Cp],Nx_lat,Nx_lat,N_trim, M_trim);
        Dp_lat_interp = reshape([LAT.Dp],Nx_lat,Nu_lat,N_trim, M_trim);
        
        Ac_lat_interp = reshape([LAT.Ac],Ni_lat,Ni_lat,N_trim, M_trim);
        Bc_lat_interp = reshape([LAT.Bc],Ni_lat,Nx_lat,N_trim, M_trim);
        Br_lat_interp = reshape([LAT.Br],Ni_lat,Nr_lat,N_trim, M_trim);
        Cc_lat_interp = reshape([LAT.Cc],Nu_lat,Ni_lat,N_trim, M_trim);
        Dc_lat_interp = reshape([LAT.Dc],Nu_lat,Nx_lat,N_trim, M_trim);
        Dr_lat_interp = reshape([LAT.Dr],Nu_lat,Nr_lat,N_trim, M_trim);
        
        Ki_lat_interp = reshape([LAT.Ki],Ni_lat,Ni_lat,N_trim, M_trim);
        Kx_lat_interp = reshape([LAT.Kx],Ni_lat,Nx_lat,N_trim, M_trim);
        Kv_lat_interp = reshape([LAT.Kv],Ni_lat,Nv_lat,N_trim, M_trim);
        
         F_lat_interp = reshape([LAT.F], Ni_lat,Nr_lat,N_trim, M_trim);
         G_lat_interp = reshape([LAT.G], Ni_lat,Nr_lat,N_trim, M_trim);
         C_lat_interp = reshape([LAT.C], Ni_lat,Nx_lat,N_trim, M_trim);
        Cv_lat_interp = reshape([LAT.Cv],Nv_lat,Nx_lat,N_trim, M_trim);
        
         Q_lat_interp = reshape([LAT.Q],2*Ni_lat, 2*Ni_lat, N_trim,M_trim);
         R_lat_interp = reshape([LAT.R],Ni_lat,Ni_lat,N_trim,M_trim);
        
         W_lat_interp = reshape([LAT.W], Nu_lat+Nv_lat,Nu_lat+Nv_lat,N_trim, M_trim);
         B_lat_interp = reshape([LAT.B], Ni_lat,       Nu_lat+Nv_lat,N_trim, M_trim);

    end % End of for kk=1:L_trim (R)
  end % End of for jj=1:M_trim (WH)
end % End of for ii=1:N_trim (UH)

% *************************************************************************
% NOTE: THIS GAIN SCHEDULE CONTROLLER/TRIM FILE NEEDS TO BE SAVED AT THE
% TOP LEVEL OF THE VEHICLE FOLDER (e.g., .../vehicles/Lift+Cruise/) IN
% ORDER TO BE FOUND/USED BY GUAM/GVS BASELINE CONTROLLER !!!!!!!
% Also, the respective setupControl.m script controls what controller/trim 
% gain schedule .mat files are used to initialize the GUAM simulation 
% (e.g. .../vehicle/Lift+Cruise/setup/setupControl.m) 
% *************************************************************************

save(fullfile(out_cntr_fpath, out_cntr_fname),...
     'UH','WH','R',...
     'Ap_lon_interp','Bp_lon_interp','Cp_lon_interp','Dp_lon_interp',...
     'Ac_lon_interp','Bc_lon_interp','Br_lon_interp',...
     'Cc_lon_interp','Dc_lon_interp','Dr_lon_interp',...
     'Ki_lon_interp','Kx_lon_interp','Kv_lon_interp',...
     'F_lon_interp','G_lon_interp','C_lon_interp','Cv_lon_interp',...
     'W_lon_interp','B_lon_interp',...
     'Ap_lat_interp','Bp_lat_interp','Cp_lat_interp','Dp_lat_interp',...
     'Ac_lat_interp','Bc_lat_interp','Br_lat_interp',...
     'Cc_lat_interp','Dc_lat_interp','Dr_lat_interp',...
     'Ki_lat_interp','Kx_lat_interp','Kv_lat_interp',...
     'F_lat_interp','G_lat_interp','C_lat_interp','Cv_lat_interp',...
     'W_lat_interp','B_lat_interp',...
     'Nx_lon','Ni_lon','Nu_lon','Nr_lon','Nv_lon', ...
     'Nx_lat','Ni_lat','Nu_lat','Nr_lat','Nv_lat', ...
     'XU0_interp')


% *************************************************************************
% Modified format (new table lookup format) below...
% *************************************************************************

% XU0_interp = zeros(N_trim, M_trim, L_trim, 25);
% 
% % pre allocate longitudinal directional blocks (modified format)
% Ap_lon_interp = zeros(N_trim, M_trim, L_trim, Nx_lon*Nx_lon);
% Bp_lon_interp = zeros(N_trim, M_trim, L_trim, Nx_lon*Nu_lon);
% Cp_lon_interp = zeros(N_trim, M_trim, L_trim, Nx_lon*Nx_lon);
% Dp_lon_interp = zeros(N_trim, M_trim, L_trim, Nx_lon*Nu_lon);
% 
% Ac_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Ni_lon);
% Bc_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Nx_lon);
% Br_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Nr_lon);
% Cc_lon_interp = zeros(N_trim, M_trim, L_trim, Nu_lon*Ni_lon);
% Dc_lon_interp = zeros(N_trim, M_trim, L_trim, Nu_lon*Nx_lon);
% Dr_lon_interp = zeros(N_trim, M_trim, L_trim, Nu_lon*Nr_lon);
% 
% Ki_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Ni_lon);
% Kx_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Nx_lon);
% Kv_lon_interp = zeros(N_trim, M_trim, L_trim, Ni_lon*Nv_lon);
% 
% F_lon_interp  = zeros(N_trim, M_trim, L_trim, Ni_lon*Nr_lon);
% G_lon_interp  = zeros(N_trim, M_trim, L_trim, Ni_lon*Nr_lon);
% C_lon_interp  = zeros(N_trim, M_trim, L_trim, Ni_lon*Nx_lon);
% Cv_lon_interp = zeros(N_trim, M_trim, L_trim, Nv_lon*Nx_lon);
% 
% Q_lon_interp  = zeros(N_trim, M_trim, L_trim, 2*Ni_lon*2*Ni_lon);
% R_lon_interp  = zeros(N_trim, M_trim, L_trim, Ni_lon,Ni_lon);
% 
% W_lon_interp  = zeros(N_trim, M_trim, L_trim, (Nu_lon+Nv_lon)*(Nu_lon+Nv_lon));
% B_lon_interp  = zeros(N_trim, M_trim, L_trim, Ni_lon*(Nu_lon+Nv_lon));
% 
% 
% % pre allocate lateral directional blocks (modified format)
% Ap_lat_interp = zeros(N_trim, M_trim, L_trim, Nx_lat*Nx_lat);
% Bp_lat_interp = zeros(N_trim, M_trim, L_trim, Nx_lat*Nu_lat);
% Cp_lat_interp = zeros(N_trim, M_trim, L_trim, Nx_lat*Nx_lat);
% Dp_lat_interp = zeros(N_trim, M_trim, L_trim, Nx_lat*Nu_lat);
% 
% Ac_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Ni_lat);
% Bc_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Nx_lat);
% Br_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Nr_lat);
% Cc_lat_interp = zeros(N_trim, M_trim, L_trim, Nu_lat*Ni_lat);
% Dc_lat_interp = zeros(N_trim, M_trim, L_trim, Nu_lat*Nx_lat);
% Dr_lat_interp = zeros(N_trim, M_trim, L_trim, Nu_lat*Nr_lat);
% 
% Ki_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Ni_lat);
% Kx_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Nx_lat);
% Kv_lat_interp = zeros(N_trim, M_trim, L_trim, Ni_lat*Nv_lat);
% 
% F_lat_interp  = zeros(N_trim, M_trim, L_trim, Ni_lat*Nr_lat);
% G_lat_interp  = zeros(N_trim, M_trim, L_trim, Ni_lat*Nr_lat);
% C_lat_interp  = zeros(N_trim, M_trim, L_trim, Ni_lat*Nx_lat);
% Cv_lat_interp = zeros(N_trim, M_trim, L_trim, Nv_lat*Nx_lat);
% 
% Q_lat_interp  = zeros(N_trim, M_trim, L_trim, 2*Ni_lat*2*Ni_lat);
% R_lat_interp  = zeros(N_trim, M_trim, L_trim, Ni_lat,Ni_lat);
% 
% W_lat_interp  = zeros(N_trim, M_trim, L_trim, (Nu_lat+Nv_lat)*(Nu_lat+Nv_lat));
% B_lat_interp  = zeros(N_trim, M_trim, L_trim, Ni_lat*(Nu_lat+Nv_lat));

% **** Store the controller outputs in the GUAM/GVS .mat format.. *********
% for ii = 1:N_trim
%   for jj = 1:M_trim
%     for kk = 1:L_trim
%       
%       XU0_interp(ii,jj,kk,:)    = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).XU0],25, 1);
% 
%       % Longitudinal
%       Ap_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ap], Nx_lon*Nx_lon, 1);
%       Bp_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Bp], Nx_lon*Nu_lon, 1);
%       Cp_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cp], Nx_lon*Nx_lon, 1);
%       Dp_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dp], Nx_lon*Nu_lon, 1);
% 
%       Ac_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ac], Ni_lon*Ni_lon, 1);
%       Bc_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Bc], Ni_lon*Nx_lon, 1);
%       Br_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Br], Ni_lon*Nr_lon, 1);
%       Cc_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cc], Nu_lon*Ni_lon, 1);
%       Dc_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dc], Nu_lon*Nx_lon, 1);
%       Dr_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dr], Nu_lon*Nr_lon, 1);
% 
%       Ki_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ki], Ni_lon*Ni_lon, 1);
%       Kx_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Kx], Ni_lon*Nx_lon, 1);
%       Kv_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Kv], Ni_lon*Nv_lon, 1);
% 
%       F_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).F],  Ni_lon*Nr_lon, 1);
%       G_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).G],  Ni_lon*Nr_lon, 1);
%       C_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).C],  Ni_lon*Nx_lon, 1);
%       Cv_lon_interp(ii,jj,kk,:) = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cv], Nv_lon*Nx_lon, 1);
% 
%       Q_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Q], (2*Ni_lon)*(2*Ni_lon), 1);
%       R_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).R], Ni_lon*Ni_lon, 1);
% 
%       W_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).W], (Nu_lon+Nv_lon)*(Nu_lon+Nv_lon), 1);
%       B_lon_interp(ii,jj,kk,:)  = reshape([LON(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).B], Ni_lon*(Nu_lon+Nv_lon), 1);
% 
%       % Lateral
%       Ap_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ap], Nx_lat*Nx_lat, 1);
%       Bp_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Bp], Nx_lat*Nu_lat, 1);
%       Cp_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cp], Nx_lat*Nx_lat, 1);
%       Dp_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dp], Nx_lat*Nu_lat, 1);
% 
%       Ac_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ac], Ni_lat*Ni_lat, 1);
%       Bc_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Bc], Ni_lat*Nx_lat, 1);
%       Br_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Br], Ni_lat*Nr_lat, 1);
%       Cc_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cc], Nu_lat*Ni_lat, 1);
%       Dc_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dc], Nu_lat*Nx_lat, 1);
%       Dr_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Dr], Nu_lat*Nr_lat, 1);
% 
%       Ki_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Ki], Ni_lat*Ni_lat, 1);
%       Kx_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Kx], Ni_lat*Nx_lat, 1);
%       Kv_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Kv], Ni_lat*Nv_lat, 1);
% 
%       F_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).F],  Ni_lat*Nr_lat, 1);
%       G_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).G],  Ni_lat*Nr_lat, 1);
%       C_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).C],  Ni_lat*Nx_lat, 1);
%       Cv_lat_interp(ii,jj,kk,:) = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Cv], Nv_lat*Nx_lat, 1);
% 
%       Q_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).Q], (2*Ni_lat)*(2*Ni_lat), 1);
%       R_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).R], Ni_lat*Ni_lat, 1);
%                                                     
%       W_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).W], (Nu_lat+Nv_lat)*(Nu_lat+Nv_lat), 1);
%       B_lat_interp(ii,jj,kk,:)  = reshape([LAT(ii+(jj-1)*N_trim+(kk-1)*N_trim*M_trim).B], Ni_lat*(Nu_lat+Nv_lat), 1);
% 
%     end % End of for kk=1:L_trim (R)
%   end % End of for jj=1:M_trim (WH)
% end % End of for ii=1:N_trim (UH)