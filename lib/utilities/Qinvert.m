function q = Qinvert(q1)

%  Return the inverse of the given quaternion(s).  Since the quaternion(s)
%  are assumed to be unit magnitude, the conjugate is actually computed.
%
%  function q = Qinvert(q1)
%
%  Usage: q = Qinvert(q1);
%
%  Description:
%
%    Return the inverse of quaternion(s) q1.  
%
%  Input:     q1 = quaternion array, either (Nx4) or (4xN).  If N = 4, 
%                   the array will be considered as 4 quaternion rows.
%
%  Outputs:   q = the inverse of q1, same dimensionality as q1.

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2006-12-15
%

[qa,n,t] = qcheck4(q1);  % check dimensionality, convert to rows
q = qa .* repmat([1 -1 -1 -1],n,1);
if t
    q = q';
end
