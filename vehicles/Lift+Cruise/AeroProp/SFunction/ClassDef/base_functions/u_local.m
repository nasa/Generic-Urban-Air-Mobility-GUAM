function [y, y_x, y_u] = u_local(x,u,w,ders)
%
% [u; v; w]_l = Ry(i)*Rx(gam)*([u; v; w] + [p; q;r] X [bx; by; bz])
%
% u_l = cos(iw)*(u + q*z - r*y) 
%       - cos(gam)*sin(iw)*(w + p*y - q*x) 
%       + sin(gam)*sin(iw)*(v - p*z + r*x)
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

% states
uu = x(:,1);
vv = x(:,2);
ww = x(:,3);
p  = x(:,4);
q  = x(:,5);
r  = x(:,6);

% inputs
i   = u(:,4);

% strip quarter chord location
bx =  w(:,1);
by =  w(:,2);
bz =  w(:,3);
gam = w(:,8);

% get the length of the data
N = size(x,1);

c_i = cos(i);
s_i = sin(i);
c_gam = cos(gam);
s_gam = sin(gam);
 
% output
y =   c_i.*(uu + q.*bz - r.*by) ...
    - c_gam.*s_i.*(ww + p.*by - q.*bx) ...
    + s_gam.*s_i.*(vv - p.*bz + r.*bx);

if ders 

  y_x = [ c_i...                               % u  
          s_gam.*s_i...                        % v 
         -c_gam.*s_i...                        % w
         -c_gam.*s_i.*by - s_gam.*s_i.*bz...   % p
          c_i.*bz + c_gam.*s_i.*bx...          % q
         -c_i.*by + s_gam.*s_i.*bx];           % r

  % u = [ T          To          del         i ]
  y_u = [ zeros(N,1) zeros(N,1) zeros(N,1) -s_i.*(uu+q.*bz-r.*by)-c_gam.*c_i.*(ww+p.*by-q.*bx)+s_gam.*c_i.*(vv-p.*bz+r.*bx) ];
else 
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
