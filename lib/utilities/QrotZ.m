function q = QrotZ(a)

%  Return quaternion(s) representing rotation(s) of _a_ about the Z axis.
%
%  Usage: q = QrotZ(a);
%
%  Input:     a = vector of rotation angle (rads), either row or column.
%
%  Outputs:   q = array of quaternion corresponding to the rotations.

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2006-12-15
%

[aa,n,t] = qcheck4(a,1);

s = cos(aa/2);
v = sin(aa/2);
q = [s, 0*v, 0*v, v];

if t
    q = q';
end
