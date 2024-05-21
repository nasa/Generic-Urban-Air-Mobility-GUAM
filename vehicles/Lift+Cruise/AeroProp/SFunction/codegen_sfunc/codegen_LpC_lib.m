function [] = codegen_LpC_lib(generateLib, copyHeaders)

arguments
  generateLib logical = true;
  copyHeaders logical = false;
end

global tiltwing;

rootDir = getenv("GVSActiveRootDir");

SimIn = evalin('base','SimIn');
tiltwing=SimIn.Model;
%rho=SimIn.Environment.Atmos.rho0;
%V_b = [100;0;0];om_b=[0;0;0];
%om_prop=[0 0 0 0 0 0 0 0 100];
%surf=[0;0;0;0];

ARGS = cell(6,1);
ARGS{1} = coder.typeof(double(0)); % rho
ARGS{2} = coder.typeof(ones(3,1)); % V_b
ARGS{3} = coder.typeof(ones(3,1)); % om_b
ARGS{4} = coder.typeof(ones(1,9)); % om_prop
ARGS{5} = coder.typeof(ones(4,1)); % surf
ARGS{6} = coder.typeof(logical(0)); % ders

if generateLib
  disp('Generating LpC_wrapper static library...');
  cfglib = coder.config('lib');
  cfglib.EnableOpenMP = false;
  cfglib.SupportNonFinite = false;
  codegen -config cfglib -report LpC_wrapper -args ARGS;
  
  objpath = sprintf('%s/obj', SimIn.vehiclepath);
  if ispc
    libext = 'lib';
    osDir = 'windows';
  elseif ismac
    libext = 'a';
    osDir = 'mac';
  elseif isunix
    libext = 'a';
    osDir = 'linux';
  end
  destlibpath = sprintf('%s/%s', objpath, osDir);
  lpclib = sprintf('%s/codegen/lib/LpC_wrapper/LpC_wrapper.%s', rootDir, libext);
  copyfile(lpclib, destlibpath);
  
else
  disp('Generating LpC_wrapper mex...');
  cfgmex = coder.config('mex');
  cfgmex.EnableOpenMP = false;
  codegen -config cfgmex -report LpC_wrapper -args ARGS;
  
  objpath = './vehicles/Lift+Cruise/obj';
  if ispc
    mexext = 'mexw64';
  elseif ismac
    mexext = 'mexmaci64';
  elseif isunix
    mexext = 'mexa64';
  end
  mexfile = sprintf('./LpC_wrapper_mex.%s', mexext);
  movefile(mexfile, objpath);
end

if copyHeaders
  % copy to headers to C MEX source folder
  disp('Copying header files...');
  files = {'LpC_wrapper.h','LpC_wrapper_terminate.h'};
  numfiles = numel(files);
  for i=1:numfiles
    srcfile = sprintf('./codegen/lib/LpC_wrapper/%s', files{i});
    copyfile(srcfile, './vehicles/Lift+Cruise/AeroProp/SFunction/codegen_sfunc');
  end
end

end

