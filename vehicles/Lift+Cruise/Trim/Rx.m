function [ R ] = Rx(x)

cx = cos(x);
sx = sin(x);
R = [1 0 0; 0 cx sx; 0 -sx cx];
