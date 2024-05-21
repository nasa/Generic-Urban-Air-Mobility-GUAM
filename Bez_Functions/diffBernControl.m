function Q = diffBernControl(P,tint,n)
%diffBernControl Generates control points of derivative curve
%   P - (vector) - vector of control points. 
%   tint - (real vector, [a,b]) - range over which curve is defined
%   n - (positve integer) - order of derivative to take
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Source: Derived from defintion. See Reference for similar derivaiton
%   Reference: Equation (5), modified for arbitrary time interval
%         @article{egerstedt2004note,
%           title={A note on the connection between Bezier curves and linear optimal control},
%           author={Egerstedt, Magnus B and Martin, Clyde F},
%           journal={IEEE transactions on automatic control},
%           volume={49},
%           number={10},
%           pages={1728--1732},
%           year={2004},
%           publisher={IEEE}
%         }
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

P = P(:);
N = size(P,1);
Ndeg = N-1;
S = prod((Ndeg):-1:(Ndeg-n+1));
% S = factorial(Ndeg)/factorial(Ndeg-n);
Q = S  / (tint(2)-tint(1)).^n * diff(P,n);


end

