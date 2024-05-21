function b = evalBernBasis(n,t)
%evalBernBasis Evaluates Bernstein basis functions of degree n at locations
%               in vector t.
%
%       n - (integer) - number of control points.
%       t - (vector) - vector of times to evaluate.
%       
%       Limitations : not recommended for most computation (speed/numerics)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Examples
%       evalBernBasis(4,[0,0.5,1]) =
%           [1 0 0 0 0;0.0625 0.25 0.375 0.25 0.0625;0 0 0 0 1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Algorithm: Definition
%   Reference:  Eqn (1)
%         @article{farouki2012bernstein,
%           title={The Bernstein polynomial basis: A centennial retrospective},
%           author={Farouki, Rida T},
%           journal={Computer Aided Geometric Design},
%           volume={29},
%           number={6},
%           pages={379--419},
%           year={2012},
%           publisher={Elsevier}
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

    B = binomCoeffs(n);
    t = t(:);    
    tm = 1 - t;    %    (1-t)
    %   Definition  B_k^n(t) := (n,k)t^k(1-t)^{n-k} for (0<=k<=n)

    T = ones(length(t), n+1);
    TM = ones(length(t), n+1);
    for j = 1:n
        T(:,j+1) = t .* T(:,j);             % powers of t
        TM(:,n+1-j) = tm .* TM(:,n+2-j);    % powers of (1-t)
    end
    
    b = B(:)'.*T.*TM;
end

