function Q = interpHermBern(wpts,tint)
% interpHermBern Solve hermite interpolation with bernstein basis fcns.
% Input:
%	points - (matrix) - vector of points to interpolate. 
%   o_continuity - (positve integer) = number of continuous derivatives + 1
%   tint - (real vector, [a,b]) - range over which curve is defined
% Output:
%   Q - (vector) - control points of interpolating curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Source: Derived from definition of derivative.
%   Limits: Number of derivatives < 25 for precision
%                                 < 80 for no NaNs
%           Improvements: possibly : Efficient Computation of Discrete Polynomial Transforms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Authors:
%       Andrew Patterson 
%       NASA Langley Research Center, D316
%       Email: andrew.patterson@nasa.gov
%   Creation: 
%       21 July 2022
%   Updates:
%       <DATE> : <DESCRIPTION>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

o_continuity = size(wpts,2);
n_der = (1:o_continuity)'-1;
% o_curve         = 2*o_continuity-1;
% S = factorial(o_curve-n_der)./factorial(o_curve);
S = [1,1./(cumprod((2*o_continuity-1):-1:(o_continuity+1)))]';
d0 = tint(2)-tint(1);
Sd = d0.^(n_der);
va = 1:o_continuity;
sn = (-1).^(va-1);
ins = (S.*Sd.*wpts(1,:)');
ins2 = (S.*Sd.*wpts(2,:)');
Q = zeros(2*o_continuity,1);
for n = va
    bn = binomCoeffs(n-1)';
    Q(n,:) = bn*ins(1:n);
    Q(end-n+1,:) = sn(1:n).*bn*ins2(1:n);
end


end

