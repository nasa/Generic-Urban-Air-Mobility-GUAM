function [y, y_x, y_u] = w_induced(x,u,w,ders,uL,uL_x,uL_u)
%
% w = 1/2 * ( -V_p + sqrt( V_p^2 + (2*T)/(A*rho) ); 
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

% number of strips
N = size(x,1);

% Thrust
T = u(:,1);
To = u(:,2);

% Propeller disk area
A = w(:,4);
Ao = w(:,5);

% air density
rho = w(:,7);

% determine the indices where there is a propeller overlap
idx = (Ao > 0) & (To > 0); % indices of overlap propellers
jdx = (A > 0) & (T > 0) & ~idx; % indices of nonoverlapped propellers
wi    = zeros(N,1);

% Propeller induced velocity, use the overlapped conditions 
% when there is an overlapped set of propellers
iSqrt = sqrt( uL(idx).^2 + 2*To(idx)./(Ao(idx).*rho(idx)));
jSqrt = sqrt( uL(jdx).^2 + 2*T(jdx)./(A(jdx).*rho(jdx)));
wi(idx) = 0.5.*( -uL(idx) + iSqrt );
wi(jdx) = 0.5.*( -uL(jdx) + jSqrt );

% outputs
y = wi;

if ders

  wi_uL = zeros(N,1);
  wi_T  = zeros(N,1);
  wi_To = zeros(N,1);

  % take the derivatives with respect to the local velocity components
  wi_uL(idx) = 0.5.*( -1 + uL(idx)./ iSqrt );
  wi_uL(jdx) = 0.5.*( -1 + uL(jdx)./ jSqrt );

  % take derivatives with respect to the changing Thrust
  wi_To(idx) =  0.5.*( 1./(Ao(idx).*rho(idx)) ./ iSqrt );
  wi_T(jdx) =  0.5.*( 1./(A(jdx).*rho(jdx)) ./ jSqrt );

  %% take the derivates with respect to the states
  wi_x1 = wi_uL.*uL_x(:,1);
  wi_x2 = wi_uL.*uL_x(:,2);
  wi_x3 = wi_uL.*uL_x(:,3);
  wi_x4 = wi_uL.*uL_x(:,4);
  wi_x5 = wi_uL.*uL_x(:,5);
  wi_x6 = wi_uL.*uL_x(:,6);

  %% take the derivates with respect to the inputs
  wi_u1 = wi_uL.*uL_u(:,1) + wi_T;
  wi_u2 = wi_uL.*uL_u(:,2) + wi_To;
  wi_u3 = wi_uL.*uL_u(:,3);
  wi_u4 = wi_uL.*uL_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ wi_x1  wi_x2  wi_x3  wi_x4  wi_x5  wi_x6 ];

  % u = [ T     To    del   i ]
  y_u = [ wi_u1 wi_u2 wi_u3 wi_u4];
else
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
