function cellInfo = PowerSystemOutBus(varargin)
% POWERSYSTEMOUTBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false;
numEng=1;
numSurf=1;
if nargin >= 1
    if islogical(varargin{1}) && varargin{1} == false 
      suppressObject = true;
    end
    if nargin == 3
        numEng=varargin{2};
        numSurf=varargin{3};
    end
end 

cellInfo = { ... 
  { ... 
    'BUS_POWER_SYSTEM_OUT', ... 
    'BusPowerSystemOut.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'CtrlSurfacePwr', numSurf, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'EnginePwr', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo);
end 
