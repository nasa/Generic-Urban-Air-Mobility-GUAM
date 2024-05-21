function b = evalBernPoly(P,t,tint,varargin)
%evalBernBasis Evaluates Bernstein polynomial at locations in vector t.
%
%       P - (vector) - vector of control points.
%       t - (vector, t\in[0,1]) - vector of times to evaluate.
%       varargin - (string) ['linear'] 'dc' 'def' 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Examples
%       evalBernPoly([0 1 1 0],0:.1:1) =
%           [0;0.27;0.48;0.63;0.72;0.75;0.72;0.63;0.48;0.27;0]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Algorithm: multiple - see below
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

    t = t(:);
    P = P(:);
    if ~isempty(tint)
        t = affineChange(t,tint);
    end
    
    if nargin>3
        switch varargin{1}
            case 'dc'
                b = deCasteljauEval(P,t);
            case 'def'
                b = defEval(P,t);
            case 'linear'
                b = linearEval(P,t);
            otherwise
                error('Unknown algorithm argument. Please use "dc", "def" or "linear".');
        end
    else
        b = linearEval(P,t);
    end 

end

function b = deCasteljauEval(P,t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Algorithm: de Casteljau
%   Complexity: 
%       Time O(n^2) 
%       Memory O(n)
%   Reference:  Eqn (13)
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

    Nc = size(P,1);
    tm = 1 - t;

    Q = P'.*ones(size(tm));

    for k = 2:Nc
        for i = 1:Nc-k+1
            Q(:,i) = tm.*Q(:,i) + t.*Q(:,i+1);
        end
    end
    b = Q(:,1);
end
function b = defEval(P,t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Algorithm: Definition
%   Complexity: 
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

    Nc = size(P,1);

    b = evalBernBasis(Nc-1,t)*P;
end
function b = linearEval(P,t)
%   Algorithm: de Casteljau
%       Complexity: 
%           Time O(n) 
%           Memory O(1)
%   Reference: Algorithm 2.1, modified according to discussion after 
%                   after Theorem 2.1.
%       @article{wozny2020linear,
%           title={Linear-time geometric algorithm for evaluating B{\'e}zier curves},
%           author={Wo{\'z}ny, Pawe{\l} and Chudy, Filip},
%           journal={Computer-Aided Design},
%           volume={118},
%           pages={102760},
%           year={2020},
%           publisher={Elsevier}
%       } 


    n = size(P,1);
    Nt = size(t,1);
    h = ones(Nt,1);
    u = 1-t;
    n1 = n;
    Q = P(1)*h;
    for k = 1:(n-1)
        h = h.*t.*(n1-k);
        h = h./(k.*u+h);
        h1 = 1-h;
        Q = h1.*Q + h.*P(k+1);
    end
    b = Q;
end