function [M] = ControlAllocation(W,B)
%CONTROLALLOCATION

M = W\B'*inv(B*inv(W)*B');

end

