function [y, y_x, y_u] = cm_local(x,u,w,aero_coefs_pp,ders,alff,alff_x,alff_u)
%
% c_m( alf_f )
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

% moment coefficient
%   need to use unmkpp/mkpp for codegen
[pp_brk, pp_coef, pp_L, pp_order, pp_dim] = unmkpp(aero_coefs_pp.cm); 
pp_mk = mkpp(pp_brk, pp_coef);
cm = ppval(pp_mk, alff);

% outputs
y = cm;

if ders

  % take some derivatives
  dcm_pp_coefs = pp_coef*[3 0 0; 0 2 0; 0 0 1; 0 0 0];
  dcm_pp_mk = mkpp(pp_brk, dcm_pp_coefs);
  cm_alff = ppval(dcm_pp_mk, alff); 

  %% take the derivates with respect to the states
  cm_x1 = cm_alff.*alff_x(:,1);
  cm_x2 = cm_alff.*alff_x(:,2);
  cm_x3 = cm_alff.*alff_x(:,3);
  cm_x4 = cm_alff.*alff_x(:,4);
  cm_x5 = cm_alff.*alff_x(:,5);
  cm_x6 = cm_alff.*alff_x(:,6);

  %% take the derivates with respect to the inputs
  cm_u1 = cm_alff.*alff_u(:,1);
  cm_u2 = cm_alff.*alff_u(:,2);
  cm_u3 = cm_alff.*alff_u(:,3);
  cm_u4 = cm_alff.*alff_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ cm_x1  cm_x2  cm_x3  cm_x4  cm_x5  cm_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ cm_u1  cm_u2  cm_u3  cm_u4];
else 
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
