function [] = Multi_Fig_Save_func(Flder_name, xscale, yscale, varargin)
% This file outputs all open Matlab figures to the specified folder and 
% saves them as matlab figures and as other filetypes specied.
% Written by Michael J. Acheson, NASA Langley Research Center.
% michael.j.acheson@nasa.gov, (757)-864-9457
% Dynamics Systems and Control Branch (DSCB D-316)

% Inputs:
%   Flder_name: absolute or relative path name where files are to go
%   xscale:     scale factor for matlab figures in x-direction (e.g., 1.4),
%               default scale is 1
%   yscale:     scale factor for matlab figures in y-direction (e.g., 1.4),
%               default scale is 1
%   varargin:   one or more file extensions to use to output the files

% Outputs:
%   files saved with .mat extension and any additional extensions as varargins
% *************************************************************************

% Determine if fldr exists, If not then create it
if (exist(Flder_name,'dir'))
    fname_base = Flder_name;
elseif (exist(fullfile(pwd,Flder_name),'dir'))
    fname_base = fullfile(pwd,Flder_name);
else
    % Make the new directory
    if strcmp(fileparts(Flder_name),'.') || strcmp(fileparts(Flder_name),'..')
        mkdir(fullfile(pwd,Flder_name));
        fname_base = fullfile(pwd,Flder_name);
    else
        mkdir(Flder_name);
        fname_base = Flder_name;
    end
end

% Get the handles of all current figures
figHandles = findobj('Type', 'figure');

fig_flag        = 0;
file_extCell    = {};
num_ext         = 0;
if nargin > 3
    for loop = 1:nargin-3
        fext_in = varargin{loop};
        switch(lower(fext_in))
            case {'.pdf','pdf'}
                file_extCell{loop}  = '.pdf';
            case {'.jpg','jpg'}
                file_extCell{loop}  = '.jpg';
            case {'.png','png'}
                file_extCell{loop}  = '.png';
            case {'.tiff','tiff'}
                file_extCell{loop}  = '.tiff';
            case {'.bmp','bmp'}
                file_extCell{loop}  = '.bmp';
            case {'fig','.fig'}
                file_extCell{loop}  = '.fig';
            otherwise
                fprintf(1,'Unknown file extension type: %s\n. File not saved!\n NOTE: add file type to Multi_Fig_Save_func.m if desired...\n', fext_in);
                return;
        end
        num_ext = num_ext +1;
    end
else
    fig_flag = 1;
    file_ext = '.fig';
    num_ext = 1;
end

for i = 1:length(figHandles)
    for j = 1:num_ext
        if ~fig_flag
            file_ext = file_extCell{j};
        end
        if fig_flag || strcmp(file_ext, '.fig')
            if ~isempty(figHandles(i).Name)
                % Create file based on name of figure
                fname = fullfile(fname_base,sprintf('%s.fig',strrep(figHandles(i).Name,' ','_')));
            else
                % Create default name for unnamed figures
                fname = fullfile(fname_base,sprintf('Figure%s.fig',num2str(figHandles(i).Number)));
            end
            figure(figHandles(i))
            savefig(figHandles(i),fname);
        else
            if ~isempty(figHandles(i).Name)
                % Create file based on name of figure
                fname2 = fullfile(fname_base,sprintf('%s%s',strrep(figHandles(i).Name,' ','_'), file_ext));
            else
                % Create default name for unnamed figures
                fname2 = fullfile(fname_base,sprintf('Figure%s%s',num2str(figHandles(i).Number), file_ext));
            end
            Print_FigSave(fname2, xscale, yscale, file_ext);
        end
    end
end
disp('Multi-Figure Save Complete');