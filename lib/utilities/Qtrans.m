function v = Qtrans(q1,v1)

%  Transform given vector(s) by given (rotation) quaternion(s).
%
%  Usage: v = Qtrans(q1,v1);
%
%  Description:
%
%    Transform a given vector(s) v1 by given (rotation) quaternion(s) q1,
%    returning the resulting vector(s) v.
%
%  Input:     q1 = quaternion(s) describing the desired rotation(s), 
%                   either (N1x4) or (4xN1).
%             v1 = original (non-rotated) vector(s), either (N2x3) or
%                   (3xN2).
%           Either N1 must equal N2, or either N1 or N2 must be 1.
%
%  Outputs:   v = the transformed (rotated) vector(s), (Nx3) or (3xN),
%                   where N is max(N1,N2).

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2006-12-15
%

[qa1,n1,t1] = qcheck4(q1,4);  % check dimensionality, convert to rows
[va1,n2,t2] = qcheck4(v1,3);  % check dimensionality, convert to rows
if n1>1 && n2>1 && n1~=n2
    disp('Inconsistent number of quaternions and vectors.');
    return
end

b1 = va1(:,1);
c1 = va1(:,2);
d1 = va1(:,3);

a2 = qa1(:,1);
b2 = qa1(:,2);
c2 = qa1(:,3);
d2 = qa1(:,4);

% compute (V1*Q1)

a3 = - b1.*b2 - c1.*c2 - d1.*d2;
b3 =   b1.*a2 + c1.*d2 - d1.*c2;
c3 =   c1.*a2 + d1.*b2 - b1.*d2;
d3 =   d1.*a2 + b1.*c2 - c1.*b2;

b2 = -b2;
c2 = -c2;
d2 = -d2;

% compute Q1~ * (V1*Q1)

v = [ a2.*b3 + b2.*a3 + c2.*d3 - d2.*c3, ...
      a2.*c3 + c2.*a3 + d2.*b3 - b2.*d3, ...
      a2.*d3 + d2.*a3 + b2.*c3 - c2.*b3 ];

if t1 && t2
    v = v';
end
