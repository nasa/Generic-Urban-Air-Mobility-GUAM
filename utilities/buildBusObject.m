function [b] = buildBusObject(varargin)
% function buildBusObject(s,prefix,name,props,options)
%% Builds bus objects from structure with prefix string.
%
% Usage: buildBusObject(s,prefix,name)
%        buildBusObject(s,prefix,name,props)
%        buildBusObject(s,prefix,name,props,options)
%

% eheim 20151009
% $Id: buildBusObject.m 2154 2017-03-27 16:25:22Z eheim $

% s = SimIn.Modeling.model0
% name = MODEL
% prefix = 'BUS_Modeling_';

% TODO: improve error checking on inputs

s      = varargin{1};
prefix = varargin{2};
name   = varargin{3};

% Set default Bus Object Properties
props.Description = '';
props.DataScope   = 'Auto';
props.HeaderFile  = '';
props.Alignment   = -1;

options.names = false;

if nargin < 3
    error('To few input arguments.')
end
if nargin > 3 
    if ~isempty(varargin{4})
        props = varargin{4};
    end
end
if nargin > 4
    options = varargin{5};
end
if nargin > 5
    error('To many input arguments.')
end

fn = fieldnames(s);% Get field names of structure

e = repmat(Simulink.BusElement,numel(fn),1);% Preallocate elements
for i = 1:numel(fn);% Cycle through field names
    x = s.(fn{i});% Pull current field
    e(i).Name = fn{i};% Set name of bus object element
    if isstruct(x);% Is this field another structure?
        % Call this function again
        if options.names;
            % Expand path name with top level prefix
            str = strcat(prefix,name,'_');
        else
            % Use top level prefix
            str = prefix;
        end

        [dt, dims, compl] = getValueAttr(x);
        % check for an array of structs
        if (dims(1) == 1)
          dim = dims(2);
        elseif (dims(2) == 1)
          dim = dims(1);
        else
          error('Unsupported dimensions, both dimensions greater than 1');
        end
        check_str = strcat(str,fn{i});
        assignin('base','check_str', check_str);
        if evalin('base', 'exist(check_str)')
            for j = 1:dim
                [k] = buildBusObject(x,str,[fn{i} '1'],props,options);
            
                exist_bus = evalin('base', check_str);
                if isequal(exist_bus, k) % Identical bus with same name already exists so don't save it..
                    e(i).DataType = strcat(str,fn{i});% Rename the data type with prefix to match bus object name
                    e(i).Dimensions = dim;% Set dimensions
                else
                    % Create a unique bus name...
                    e(i).DataType = strcat(str,[fn{i} '1']);% Rename the data type with prefix to match bus object name
                    assignin('base',strcat(str,[fn{i} '1']), k);
                    e(i).Dimensions = dim;% Set dimensions
                end
            end
        else
            for j = 1:dim
                [k] = buildBusObject(x,str,fn{i},props,options);
                assignin('base',check_str, k);
            end
            e(i).DataType = strcat(str,fn{i});% Rename the data type with prefix to match bus object name
        end
    else
        [dt, dims, compl] = getValueAttr(x);
        e(i).Complexity = compl;
        if (dims(1) == 1)
          dim = dims(2);
        elseif (dims(2) == 1)
          dim = dims(1);
        else
            dim = dims;
        end
        e(i).Dimensions = dim;% Set dimensions
        e(i).DataType   = dt;% Set Data type
    end
end
b = Simulink.Bus;
b.Description = props.Description;
b.DataScope   = props.DataScope;
b.HeaderFile  = props.HeaderFile;
b.Alignment   = props.Alignment;
b.Elements = e;
% assignin('base',[prefix name],b);% % Rename the bus object with prefix
end


% Get DataType, Dims and Complexity
function [dt, dims, compl] = getValueAttr(val)
p = Simulink.Parameter;
dataValue = val;
if isa(val,'timeseries')
    dataValue = val.Data;
end
p.Value = dataValue;
compl = p.Complexity;
dt = p.DataType;
if isequal(dt, 'auto')
    dt = 'double';
end
if isa(val,'timeseries')
    dims = Simulink.SimulationData.TimeseriesUtil.getSampleDimensions(val);
else
    dims = p.Dimensions;
end
end

