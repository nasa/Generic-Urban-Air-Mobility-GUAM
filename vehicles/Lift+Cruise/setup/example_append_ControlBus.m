function append_ControlBus(varargin)

cellInfo = ControlBus(false,10,9);
numElem=length(cellInfo{1}{6});

% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 
newElem = {'AuxCmd', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''};

cellInfo{1}{6}{numElem+1}=newElem;

% Create bus objects in the MATLAB base workspace 
Simulink.Bus.cellToObject(cellInfo) 
