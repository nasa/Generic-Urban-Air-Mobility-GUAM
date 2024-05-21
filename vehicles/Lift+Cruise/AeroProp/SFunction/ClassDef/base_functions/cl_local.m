function [y, y_x, y_u] = cl_local(x,u,w,aero_coefs_pp,ders,alff,alff_x,alff_u)
%
% cl = cl(alf_f) 
%
% Inputs: 
%
%     x: Nx6 array of dynamic state
%        x = [ u v w p q r ] 
%
%     u: Nx4 array of inputs
%        u = [ T To del_f i ]
%
%     w: Nx8 array of strip parameters 
%        w = [ bx by bz A Ao S rho gam]
%   
%     aero_coefs_pp: aerodynamic polynomial fit coefficients
%
% Outputs:
%
%     y: Nx1 value of functional
%  
%     y_x: Nx6 array of derivative with respect to the state
%          y_x = [ dydu dydv dydw dydp dydq dydr ]
%
%     y_u: Nx4 array of derivative with respect to the input
%          y_x = [ dydT dydT0 dyddel_f dydi ]
%
%
%                    Y
%                     \
%                      \
%                        __
%                       \  \     _ _
%                        \**\ ___\/ \
%                X --- X*#####*+^^\_\
%                        o/\  \
%                           \__\
%                         |
%                         |
%                         Z
%
% state vector x:
%   u - velocity in body X-axis, m/s (ft/s)
%   v - velocity in body Y-axis, m/s (ft/s)
%   w - velocity in body Z-axis, m/s (ft/s)
%   p - angular velocity about body X-axis, rad/s 
%   q - angular velocity about body Y-axis, rad/s
%   r - angular velocity about body Z-axis, rad/s
%
% input vector u:
%     T - thrust of the non-overlapped propeller, N (lbf)
%    To - thrust of the overlapped propeller, N (lbf)
% del_f - delfection angle of the flap, rad
%     i - tilt angle of the wing, rad
%
% exogenous input vector w:
%  bx - strip c/4 x location with respect to center of mass, m (ft)
%  by - strip c/4 y location with respect to center of mass, m (ft)
%  bz - strip c/4 z location with respect to center of mass, m (ft)
%   A - non-overlapped propeller strip area, m^2 (ft^2)
%  Ao - overlapped propeller strip area, m^2 (ft^2)
%   S - wing strip area, m^2 (ft^2)
% rho - air density, kg/m^3 (slugs/ft^3)
% gam - wing dihedral angle  

% lift coefficient
%   need to use unmkpp/mkpp for codegen
[pp_brk, pp_coef, pp_L, pp_order, pp_dim] = unmkpp(aero_coefs_pp.cl); 
pp_mk = mkpp(pp_brk, pp_coef);
cl = ppval(pp_mk, alff);

% outputs
y = cl;

if ders

  % take some derivatives
  dcl_pp_coefs = pp_coef*[3 0 0; 0 2 0; 0 0 1; 0 0 0];
  dcl_pp_mk = mkpp(pp_brk, dcl_pp_coefs);
  cl_alff = ppval(dcl_pp_mk, alff); 

  %% take the derivates with respect to the states
  cl_x1 = cl_alff.*alff_x(:,1);
  cl_x2 = cl_alff.*alff_x(:,2);
  cl_x3 = cl_alff.*alff_x(:,3);
  cl_x4 = cl_alff.*alff_x(:,4);
  cl_x5 = cl_alff.*alff_x(:,5);
  cl_x6 = cl_alff.*alff_x(:,6);

  %% take the derivates with respect to the inputs
  cl_u1 = cl_alff.*alff_u(:,1);
  cl_u2 = cl_alff.*alff_u(:,2);
  cl_u3 = cl_alff.*alff_u(:,3);
  cl_u4 = cl_alff.*alff_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ cl_x1  cl_x2  cl_x3  cl_x4  cl_x5  cl_x6 ];

  % u = [ T     T0    del   i ]
  y_u = [ cl_u1 cl_u2 cl_u3 cl_u4 ];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
