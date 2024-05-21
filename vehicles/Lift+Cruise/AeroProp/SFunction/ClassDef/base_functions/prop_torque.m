function [y, y_x, y_u] = prop_torque(x,u,w,prop_coef,ders)
%
% Q = c_Q * rho * (om/(2*pi))^2 * D^5
%
% Inputs: 
%
%     x: Nx6 array of dynamic state
%        x = [ u v w p q r ] 
%
%     u: Nx1 array of prop speeds 
%        u = [ om ]
%
%     w: Nx8 array of strip parameters 
%        w = [ bx by bz ex ey ez D rho ]
%   
%     prop_coefs_pp: propeller polynomial fit coefficients
%
% Outputs:
%
%     y: Nx1 value of functional
%  
%     y_x: Nx6 array of derivative with respect to the state
%          y_x = [ dydu dydv dydw dydp dydq dydr ]
%
%     y_u: Nx1 array of derivative with respect to the input
%          y_x = [ dydom ]
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
%     om - speed of the propeller, rad/s
%
% parameter input vector w:
%  bx - prop x location with respect to center of mass, m (ft)
%  by - prop y location with respect to center of mass, m (ft)
%  bz - prop z location with respect to center of mass, m (ft)
%  ex - prop orientation in the body frame x 
%  ey - prop orientation in the body frame y
%  ez - prop orientation in the body frame z
%   D - propeller diameter, m (ft)
% rho - air density, kg/m^3 (slugs/ft^3)


ex   = w(:,4);
ey   = w(:,5);
ez   = w(:,6);
D    = w(:,7);
rho  = w(:,8);
%om   = u(:,1);

% local flow conditions 
% make a dummy u and w vector
Np = numel(u);
if (Np > 0)
  om = u(1:Np,1);
else
  Np = 1;
  om = ones(1,1);
end
xt = x(1:Np,1:6);
wt = zeros(Np,8);
ut = zeros(Np,4);
%wt(:,1:3) = w(:,1:3);
wt(1:Np,1:3) = w(1:Np,1:3);

[uL uL_x uL_u] = u_local(xt,ut,wt,ders);
[vL vL_x vL_u] = v_local(xt,ut,wt,ders);
[wL wL_x wL_u] = w_local(xt,ut,wt,ders);

% perpendicular velocity
vp    = ex.*uL+ey.*vL+ez.*wL;

% effective advance ratio
J    =  2*pi*vp./(D.*om);

% thrust coefficient
cP   =   prop_coef(1:Np,1).*J.^2 + prop_coef(1:Np,2).*J + prop_coef(1:Np,3);
cQ    = -cP/(2*pi);

% catch some bad divide by zeros
zidx = (om < 0.1);
cQ(zidx) = 0;

% torque
Q    = cQ.*rho.*(om./(2*pi)).^2.*D.^5;

% outputs
y = Q;

if ders 

  % perpendicular velocity
  vp_uL = ex;
  vp_vL = ey;
  vp_wL = ez;

  % effective advance ratio
  J_vp =  2*pi   ./(D.*om);
  J_om = -2*pi*vp./(D.*om.^2);

  % thrust coefficient
  cP_J = 2*prop_coef(:,1).*J    + prop_coef(:,2);
  cQ_J  = -cP_J/(2*pi);

  % catch some bad divide by zeros
  cQ_J(zidx) = 0;
  J_om(zidx) = 0;
  J_vp(zidx) = 0;

  Q_cQ =     rho.*(om./(2*pi)).^2.*D.^5;
  Q_om = Q_cQ.*cQ_J.*J_om + 2/(2*pi).*rho.*(om./(2*pi)).*D.^5.*cQ;

  %% take the derivates with respect to the states
  Q_x1 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,1)+vp_vL.*vL_x(:,1)+vp_wL.*wL_x(:,1));
  Q_x2 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,2)+vp_vL.*vL_x(:,2)+vp_wL.*wL_x(:,2));
  Q_x3 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,3)+vp_vL.*vL_x(:,3)+vp_wL.*wL_x(:,3));
  Q_x4 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,4)+vp_vL.*vL_x(:,4)+vp_wL.*wL_x(:,4));
  Q_x5 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,5)+vp_vL.*vL_x(:,5)+vp_wL.*wL_x(:,5));
  Q_x6 = Q_cQ.*cQ_J.*J_vp.*(vp_uL.*uL_x(:,6)+vp_vL.*vL_x(:,6)+vp_wL.*wL_x(:,6));

  %% take the derivates with respect to the inputs
  Q_u1 = Q_om;

  % x = [ u      v      w      p      q      r ]
  y_x = [ Q_x1  Q_x2  Q_x3  Q_x4  Q_x5  Q_x6 ];

  % u = [ om    ]
  y_u = [ Q_u1 ];
else 
  y_x = zeros(Np,6);
  y_u = zeros(Np,1);
end
