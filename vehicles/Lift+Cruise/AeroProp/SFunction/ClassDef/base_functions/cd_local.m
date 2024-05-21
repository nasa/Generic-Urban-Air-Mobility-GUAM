function [y, y_x, y_u] = cd_local(x,u,w,aero_coefs_pp,ders,alff,alff_x,alff_u)
%
% c_d( alf_f )
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


% drag coefficient
%   need to use unmkpp/mkpp for codegen
[pp_brk, pp_coef, pp_L, pp_order, pp_dim] = unmkpp(aero_coefs_pp.cd); 
pp_mk = mkpp(pp_brk, pp_coef);
cd = ppval(pp_mk, alff);

% outputs
y = cd;

if ders

  % take some derivatives
  dcd_pp_coefs = pp_coef*[3 0 0; 0 2 0; 0 0 1; 0 0 0];
  dcd_pp_mk = mkpp(pp_brk, dcd_pp_coefs);
  cd_alff = ppval(dcd_pp_mk, alff); 

  %% take the derivates with respect to the states
  cd_x1 = cd_alff.*alff_x(:,1);
  cd_x2 = cd_alff.*alff_x(:,2);
  cd_x3 = cd_alff.*alff_x(:,3);
  cd_x4 = cd_alff.*alff_x(:,4);
  cd_x5 = cd_alff.*alff_x(:,5);
  cd_x6 = cd_alff.*alff_x(:,6);

  %% take the derivates with respect to the inputs
  cd_u1 = cd_alff.*alff_u(:,1);
  cd_u2 = cd_alff.*alff_u(:,2);
  cd_u3 = cd_alff.*alff_u(:,3);
  cd_u4 = cd_alff.*alff_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ cd_x1  cd_x2  cd_x3  cd_x4  cd_x5  cd_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ cd_u1  cd_u2  cd_u3  cd_u4];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
