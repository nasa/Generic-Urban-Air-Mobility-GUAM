function [y, y_x, y_u] = q_slipstream(x,u,w,ders,Vs,Vs_x,Vs_u)
%
% q_s = 1/2*rho*Vs^2;
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
%               X ---  X*#####*+^^\_\
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

% air density
rho = w(:,7);

% slipstream dynamic pressure
qs = 0.5*rho.*Vs.^2; 
% outputs
y = qs;

if ders 

  % take some derivatives
  qs_Vs = rho.*Vs;

  %% take the derivates with respect to the states
  qs_x1 = qs_Vs.*Vs_x(:,1);
  qs_x2 = qs_Vs.*Vs_x(:,2);
  qs_x3 = qs_Vs.*Vs_x(:,3);
  qs_x4 = qs_Vs.*Vs_x(:,4);
  qs_x5 = qs_Vs.*Vs_x(:,5);
  qs_x6 = qs_Vs.*Vs_x(:,6);

  %% take the derivates with respect to the inputs
  qs_u1 = qs_Vs.*Vs_u(:,1);
  qs_u2 = qs_Vs.*Vs_u(:,2);
  qs_u3 = qs_Vs.*Vs_u(:,3);
  qs_u4 = qs_Vs.*Vs_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ qs_x1  qs_x2  qs_x3  qs_x4  qs_x5  qs_x6 ];

  % u = [ T     To    del   i ]
  y_u = [ qs_u1 qs_u2 qs_u3 qs_u4];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
