function q = QmultSeq(q1,varargin)

%  Multiply an arbitrary number of quaternions.
%
%  function q = QmultSeq(q1,q2,...,qn)
%
%  Usage: q = QmultSeq(q1,q2,...,qn);
%
%  Description:
%
%    Multiply a sequence of quaternions, returning the result.  Note that the order 
%    of the arguments is important, as quaternion multiplication is NOT COMMUTATIVE.
%
%  Input:     q1,...,qn = the quaternions to be multiplied, in order.
%
%  Outputs:   q = the resulting quaternion (product).

%
%    Calls:
%      Qmult.
%
%    Author:  Stephen D. Derry      2007-03-30
%

q = q1;
for i = 2:nargin
    q = Qmult(q, varargin{i-1});
end
