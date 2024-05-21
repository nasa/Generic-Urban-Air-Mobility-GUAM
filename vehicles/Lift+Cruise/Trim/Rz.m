function [ R ] = Rz(x)

cx = cos(x);
sx = sin(x);
R = [cx sx 0;-sx cx 0 ; 0 0 1];
