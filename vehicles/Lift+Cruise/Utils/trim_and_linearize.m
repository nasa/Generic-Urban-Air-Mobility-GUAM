Vtot = 20* SimIn.Units.knot;
verbose = 0;
t = [0:60];

clear SimOP;

fprintf(1, 'Vtot = %5.2f knots\n', Vtot / SimIn.Units.knot);

fprintf(1, 'Trimming...\n');
SimOP.IC = trim_LC(SimIn, struct('tas', Vtot, 'gamma', 0), verbose);
SimOP.EOM = SimIn.EOM;

return

fprintf(1, 'Linearizing...\n');
[SimOP.ss_quat, SimOP.ss_euler] = linearize_LC(SimIn, SysTypeEnum.BasicPlant);

% disable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('off', id);

% OP structure required for LinearSim model
OP = SimOP;

fprintf(1, 'Simulating...\n');
sim('LinearSim', t);
linearSimOut = LinSimOut;

% re-enable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('on', id);
  
clear id verbose numOp t Vtot
