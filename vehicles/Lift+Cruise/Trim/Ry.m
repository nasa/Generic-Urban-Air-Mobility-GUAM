function [ R ] = Ry(x)

cx = cos(x);
sx = sin(x);
R = [cx 0 -sx; 0 1 0; sx 0 cx];
