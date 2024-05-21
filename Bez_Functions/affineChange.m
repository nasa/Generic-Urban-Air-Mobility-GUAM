function out = affineChange(in,tint)
%AFFINECHANGE shift and scale values in [a,b] to [0,1] 
%   params - [a,b]
a = tint(1);
b = tint(2);
%a + (b-a)*in
out = (in-a)/(b-a);
end

