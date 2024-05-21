
V_sweep = [80:10:200] * SimIn.Units.knot;
numOp = numel(V_sweep);
verbose = 0;
t = [0:60];

clear SimOP;

for idxOp = 1:numOp
  Vtot = V_sweep(idxOp);
  fprintf(1, 'Vtot = %5.2f knots\n', Vtot / SimIn.Units.knot);

  fprintf(1, 'Trimming...\n');
  SimOP(idxOp).IC = trim_LC(SimIn, struct('tas', Vtot, 'gamma', 0), verbose);
  SimOP(idxOp).EOM = SimIn.EOM;

  fprintf(1, 'Linearizing...\n');
  [SimOP(idxOp).ss_quat, SimOP(idxOp).ss_euler] = linearize_LC(SimIn, SysTypeEnum.BasicPlant);

  % disable warnings about missing 'From' for 'Goto'
  id = 'Simulink:blocks:MatchingFromNotFound';
  warning('off', id);

  fprintf(1, 'Simulating...\n');
  % OP structure required for LinearSim model
  OP = SimOP(idxOp);
  sim('LinearSim', t);
  linearSimOut(idxOp) = LinSimOut;

  % re-enable warnings about missing 'From' for 'Goto'
  id = 'Simulink:blocks:MatchingFromNotFound';
  warning('on', id);
end

clear id verbose numOp t Vtot