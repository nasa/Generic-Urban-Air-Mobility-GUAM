function cellInfo = TrimInputBus(varargin)
% TRIMINPUTBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false;
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
  if nargin == 3
    numEngines=varargin{2};
    numSurf=varargin{3};
  else
    error('Invalid input argument(s) encountered'); 
  end 
end 

cellInfo = { ... 
  { ... 
    'BUS_TRIM_INPUT', ... 
    'BusTrimInput.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Engines', numEngines, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Surfaces', numSurf, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}';  

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo);
end 
