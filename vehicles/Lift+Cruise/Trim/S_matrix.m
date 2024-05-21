function [ S ] = S_matrix(phi, theta)

c_th = cos(theta);
t_th = tan(theta);

s_phi = sin(phi);
c_phi = cos(phi);

S = [1  s_phi*t_th  c_phi*t_th ; 
     0  c_phi      -s_phi      ; 
     0  s_phi/c_th  c_phi/c_th];
