function [qa,n,t] = qcheck4(q,K)

%
%   Check dimensionality of input array, force into row format (NxK).
%
%       [qa,n,t] = qcheck4(q,K);
%
%       q = array of input vectors, either (NxK) or (KxN).
%       K = (optional) expected size of individual vectors.  Since this
%           routine is primarily intended for processing arrays of
%           quatenions, K defaults to 4.
%
%       qa = same array of quaternions, forced into row format (NxK).
%       n = number of rows in qa.
%       t = non-zero iff q was originally in column format (KxN).
%
%   Note that if q is (KxK), i.e. N=K, then q will be assumed to be in row
%   format.

%   2010-02-06  S.Derry

if nargin < 2
    K = 4;
end

if ndims(q) > 2
    disp('Quaternion array with >2 dimsnsions not supported.');
    return
end

[nr,nc] = size(q);

if nc == K
    qa = q;
    n = nr;
    t = 0;
    
elseif nr == K
    qa = q';
    n = nc;
    t = 1;
    
else
    disp(['Input array has no dimension of ' num2str(K) '.']);
end
