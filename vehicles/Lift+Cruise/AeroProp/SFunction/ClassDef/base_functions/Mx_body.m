function [y, y_x, y_u] = Mx_body(x,u,w,aero_coefs_pp,ders)
%
% Mx = Fz*by - Fy*bz;
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

% y location of strip
by = w(:,2);
bz = w(:,3);

% forces
[Fz Fz_x Fz_u] = Fz_body(x,u,w,aero_coefs_pp,ders);
[Fy Fy_x Fy_u] = Fy_body(x,u,w,aero_coefs_pp,ders);

% Mx 
Mx = Fz.*by - Fy.*bz;
% outputs
y = Mx;

if ders 

  % take some derivatives
  Mx_Fz =  by;
  Mx_Fy = -bz;

  %% take the derivates with respect to the states
  Mx_x1 = Mx_Fz.*Fz_x(:,1) + Mx_Fy.*Fy_x(:,1);
  Mx_x2 = Mx_Fz.*Fz_x(:,2) + Mx_Fy.*Fy_x(:,2);
  Mx_x3 = Mx_Fz.*Fz_x(:,3) + Mx_Fy.*Fy_x(:,3);
  Mx_x4 = Mx_Fz.*Fz_x(:,4) + Mx_Fy.*Fy_x(:,4);
  Mx_x5 = Mx_Fz.*Fz_x(:,5) + Mx_Fy.*Fy_x(:,5);
  Mx_x6 = Mx_Fz.*Fz_x(:,6) + Mx_Fy.*Fy_x(:,6);

  %% take the derivates with respect to the inputs
  Mx_u1 = Mx_Fz.*Fz_u(:,1) + Mx_Fy.*Fy_u(:,1); 
  Mx_u2 = Mx_Fz.*Fz_u(:,2) + Mx_Fy.*Fy_u(:,2);
  Mx_u3 = Mx_Fz.*Fz_u(:,3) + Mx_Fy.*Fy_u(:,3);
  Mx_u4 = Mx_Fz.*Fz_u(:,4) + Mx_Fy.*Fy_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ Mx_x1  Mx_x2  Mx_x3  Mx_x4  Mx_x5  Mx_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ Mx_u1  Mx_u2  Mx_u3  Mx_u4];
else
  y_x = [];
  y_u = [];
end
