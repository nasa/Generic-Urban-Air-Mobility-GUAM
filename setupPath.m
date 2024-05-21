disp('Default path setup');

% Restore path to default
restoredefaultpath;

% save root dir off to ease directory related items further along in the
% process (mainly to get rid of several relative pathing items).
setenv("GVSActiveRootDir", pwd);

% add path for libs, utilities, etc
% path(genpath('AutoCodeModels'), path);
path(genpath('ClassDef'), path);
path(genpath('Environment'), path);
addpath ./lib;

path(genpath('./lib/utilities'),path);

path(genpath('./lib/Control'), path);
path(genpath('utilities'), path);
path(genpath('setup'), path);
path(genpath('Exec_Scripts'), path);
addpath ./vehicles;
addpath ./Bez_Functions;
addpath ./Challenge_Problems/;


