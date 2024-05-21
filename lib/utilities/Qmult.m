function q = Qmult(q1,q2)

%  Multiply two sets of quaternions.
%
%  function q = Qmult(q1,q2)
%
%  Usage: q = Qmult(q1,q2);
%
%  Description:
%
%    Multiply two sets of quaternions, returning the result: q = q1 * q2.  Note that the order 
%    of the arguments is important, as quaternion multiplication is NOT COMMUTATIVE.
%
%  Input:     q1 = first quaternion (multiplicand) array, either (N1x4) or (4xN1).
%             q2 = second quaternion (multiplier), either (N2x4) or (4xN2).
%       If N1 ~= N2, then either N1 or N2 must be 1.
%
%  Outputs:   q = the resulting quaternion (product), (Nx4), where N is
%               max(N1,N2), unless both q1 and q2 are (4xn), in which case
%               q will be (4xN).

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2006-12-15
%

[qa1,n1,t1] = qcheck4(q1);  % check dimensionality, convert to rows
[qa2,n2,t2] = qcheck4(q2);  % check dimensionality, convert to rows
if n1>1 && n2>1 && n1~=n2
    disp('Inconsistent number of quaternions to multiply.');
    return
end

a1 = qa1(:,1);
b1 = qa1(:,2);
c1 = qa1(:,3);
d1 = qa1(:,4);

a2 = qa2(:,1);
b2 = qa2(:,2);
c2 = qa2(:,3);
d2 = qa2(:,4);

q = [ a1.*a2 - b1.*b2 - c1.*c2 - d1.*d2, ...
      a1.*b2 + b1.*a2 + c1.*d2 - d1.*c2, ...
      a1.*c2 + c1.*a2 + d1.*b2 - b1.*d2, ...
      a1.*d2 + d1.*a2 + b1.*c2 - c1.*b2 ];

if t1 && t2
    q = q';
end
