function [target] = setupTargets(SimIn)

Vtot = 200 * SimIn.Units.knot;

%target = struct('tas', Vtot,'beta',0,'gamma',0,'gndtrack',0,'roll',5);  %need to include target for ay..
%target = struct('tas', Vtot,'gamma',0,'gndtrack',0,'roll',5);  %need to include target for ay..
target = struct('tas', Vtot,'gamma',0);  %need to include target for ay..

end

