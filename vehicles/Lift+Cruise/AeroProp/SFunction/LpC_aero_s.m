function semi_aero_s(block)

%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BUILD THE AIRCRAFT AND SAVE TO USER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Build the aircraft
tiltwing = block.DialogPrm(1).Data;

%% Set the number of inputs and outputs
NPw = tiltwing.WingProp.NP;
NPt = tiltwing.Tail.NP;
NPp = length(tiltwing.Prop); 

NU = sum([NPw NPt NPp 6]); % number of inputs
NX = 6; % number of states

%% Save the tiltwing object to User data to allow acces in other blocks
set_param(block.BlockHandle, 'UserData', tiltwing);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Register number of ports
block.NumInputPorts  = 3;
block.NumOutputPorts = 6;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions        =  NX;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

block.InputPort(3).Dimensions        = NU;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% Override output port properties
% Total Force/Moment
block.OutputPort(1).Dimensions  = 6;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Aerodynamic Total Force/Moment
block.OutputPort(2).Dimensions  = 6;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';

% Propulsion Total Force/Moment
block.OutputPort(3).Dimensions  = 6;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';

% Propulsion torque/thrust per engine
block.OutputPort(4).Dimensions  = [2,NPp];
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';

% mass
block.OutputPort(5).Dimensions  = 1;
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';

% Inertia matrix
block.OutputPort(6).Dimensions  = [3,3];
block.OutputPort(6).DatatypeID  = 0; % double
block.OutputPort(6).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 1;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%
% SetInputPortSamplingMode:
%   Functionality    : Check and set input and output port 
%                      attributes and specify whether the port is operating 
%                      in sample-based or frame-based mode
%   C-Mex counterpart: mdlSetInputPortFrameData.
%   (The DSP System Toolbox is required to set a port as frame-based)
%
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
function Start(block)

block.Dwork(1).Data = 0;

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

rho = block.InputPort(1).Data;
X = block.InputPort(2).Data;
U = block.InputPort(3).Data;

%% retrieve tiltwing object
tiltwing = block.DialogPrm(1).Data;

% Define the number of states
NPw = tiltwing.WingProp.NP;
NPt = tiltwing.Tail.NP;
NPp = length(tiltwing.Prop); 

% states
Vb    = X(1:3);
om   = X(4:6);

% inputs 
om_w  = U(1:NPw)';
om_t  = U(NPw+1:NPw+NPt);
om_p  = U(NPw+NPt+1:NPw+NPt+NPp);
del_a = U(NPw+NPt+NPp+1);
del_f = U(NPw+NPt+NPp+2);
del_e = U(NPw+NPt+NPp+3);
del_r = U(NPw+NPt+NPp+4);
iw    = U(NPw+NPt+NPp+5);
it    = U(NPw+NPt+NPp+6);


%%%%% Set the tiltwing inputs %%%%%

%% speed of Wing Propellers
if ~isempty(om_w)
  tiltwing.om_w = om_w;
end

%% speed of Tail Propellers
if ~isempty(om_t)
  tiltwing.om_t = om_t;
end

%% speed of Extra Propellers
if ~isempty(om_p)
  tiltwing.om_p = om_p;
end

%% Deflecting surfaces
tiltwing.del_a = del_a; % aileron
tiltwing.del_f = del_f; % flap
tiltwing.del_e = del_e; % elevator
tiltwing.del_r = del_r; % rudder

%% Wing and tail tilt angles
% tiltwing.i_w = iw; % wing tilt
% tiltwing.i_t = it; % tail tilt
tiltwing.i_w = 0; % wing tilt
tiltwing.i_t = 0; % tail tilt


%%%%% Tiltwing Aerodynamic %%%%%
ders = false; % turn off derivative calculations
%ders = true;  % turn on derivative calculations
tiltwing = tiltwing.aero(rho, Vb, om, ders);

Fb = tiltwing.total_Fb();
Mb = tiltwing.total_Mb();

propTorq   = tiltwing.Qp;
propThrust = tiltwing.Tp;

aero_Fb = tiltwing.aero_Fb();
aero_Mb = tiltwing.aero_Mb();

prop_Fb = tiltwing.prop_Fb();
prop_Mb = tiltwing.prop_Mb();

%%%%% Set the outputs %%%%%

block.OutputPort(1).Data = [Fb; Mb]; % aerodynamic forces/moments in the body frame
block.OutputPort(2).Data = [aero_Fb; aero_Mb]; % aerodynamic forces/moments in the body frame
block.OutputPort(3).Data = [prop_Fb; prop_Mb]; % propulsion forces/moments in the body frame

block.OutputPort(4).Data = [propTorq; propThrust]; % propulsion torque/thrust per engine
block.OutputPort(5).Data = tiltwing.mass; % mass of aircraft
block.OutputPort(6).Data = tiltwing.I; % Inertia matrix of aircraft



%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)


%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode  = fd;
  block.OutputPort(3).SamplingMode  = fd;
  block.OutputPort(4).SamplingMode  = fd;
  block.OutputPort(5).SamplingMode  = fd; 
  block.OutputPort(6).SamplingMode  = fd;
%endfunction

