function [y, y_x, y_u] = alf_wing(x,u,w,ders)
%
% alf_w = arctan( w / u ) + i; 
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

% get dimensions of the input
N = size(x,1);

% X and Z components of the velocity
uu = x(:,1);
ww = x(:,3);

% tilt angle of the wing relative to body
i = u(:,4);

% local velocity 
alfw = atan2(ww,uu) + i; 

% outputs
y = alfw;

if ders

  % take the derivatives with respect to the local velocity components
  alfw_u = 1./(1+(ww./uu).^2).*(-ww./uu.^2);
  alfw_w = 1./(1+(ww./uu).^2).*(1./uu);
  alfw_i = ones(N,1);

  %% take the derivates with respect to the states
  alfw_x1 = alfw_u;
  alfw_x2 = zeros(N,1);
  alfw_x3 = alfw_w;
  alfw_x4 = zeros(N,1);
  alfw_x5 = zeros(N,1);
  alfw_x6 = zeros(N,1);

  %% take the derivates with respect to the inputs
  alfw_u1 = zeros(N,1);
  alfw_u2 = zeros(N,1);
  alfw_u3 = zeros(N,1);
  alfw_u4 = alfw_i;

  % x = [ u        v        w        p        q        r ]
  y_x = [ alfw_x1  alfw_x2  alfw_x3  alfw_x4  alfw_x5  alfw_x6 ];

  % u = [ T        To       del      i ]
  y_u = [ alfw_u1  alfw_u2  alfw_u3 alfw_u4 ];
else
  y_x = [];
  y_u = [];
end
