function [y, y_x, y_u] = My_local(x,u,w,ders,...
                             cm,cm_x,cm_u,...
                             qs,qs_x,qs_u)
%
% My = cm*S*qs
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

% wing area
S = w(:,6);

% moment
My = cm.*S.*qs;
% outputs
y = My;

if ders

  % take some derivatives
  My_cm =     S.*qs;
  My_qs = cm.*S;

  %% take the derivates with respect to the states
  My_x1 = My_cm.*cm_x(:,1) + My_qs.*qs_x(:,1);
  My_x2 = My_cm.*cm_x(:,2) + My_qs.*qs_x(:,2);
  My_x3 = My_cm.*cm_x(:,3) + My_qs.*qs_x(:,3);
  My_x4 = My_cm.*cm_x(:,4) + My_qs.*qs_x(:,4);
  My_x5 = My_cm.*cm_x(:,5) + My_qs.*qs_x(:,5);
  My_x6 = My_cm.*cm_x(:,6) + My_qs.*qs_x(:,6);

  %% take the derivates with respect to the inputs
  My_u1 = My_cm.*cm_u(:,1) + My_qs.*qs_u(:,1);
  My_u2 = My_cm.*cm_u(:,2) + My_qs.*qs_u(:,2);
  My_u3 = My_cm.*cm_u(:,3) + My_qs.*qs_u(:,3);
  My_u4 = My_cm.*cm_u(:,4) + My_qs.*qs_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ My_x1  My_x2  My_x3  My_x4  My_x5  My_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ My_u1  My_u2  My_u3  My_u4];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
