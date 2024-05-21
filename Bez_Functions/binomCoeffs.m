function B = binomCoeffs(n)
%binomCoeff Computes binomial coefficients for bernstein basis polynomial  
%           generation. Returns (n+1)^th row of Pascal's triangle.
%
%       n - (integer) - number of control points.
%       
%       Limits : large n (>25) -> numeric errors, computation time
%                   see reference for options
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Examples
%       binomCoeffs(1) = [1;1]
%       binomCoeffs(10) = [1;10;45;120;210;252;210;120;45;10;1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Algorithm: Multiplicative formula for binomial coefficients
%       properties:
%           time    O(N)
%           space   O(1)
%   Reference:
%         article{araujo2021fast,
%             title={Fast computation of binomial coefficients},
%             author={Araujo, Leonardo C and Sans{\~a}o, Jo{\~a}o PH and Vale-Cardoso, Adriano S},
%             journal={Numerical Algorithms},
%             volume={86},
%             number={2},
%             pages={799--812},
%             year={2021},
%             publisher={Springer}
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

    B = ones(n+1, 1);
    for j = 1:ceil(n/2) % row is symmetric
        B(j+1) = B(j)*(n+1-j)/j;    %
        B(n+1-j) = B(j+1);
    end
    
end

