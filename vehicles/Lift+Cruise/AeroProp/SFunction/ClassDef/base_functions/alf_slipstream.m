function [y, y_x, y_u] = alf_slipstream(x,u,w,ders,...
                                      uL,uL_x,uL_u,...
                                      wL,wL_x,wL_u,...
                                      wi,wi_x,wi_u)
%
% alf_s = arctan( ( wL / (uL + 2*w_i) );
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

% Slipstream angle of attack 
alfs = atan2(wL, uL + 2*wi); 

% outputs
y = alfs;

if ders

  % NEED TO CHECK HERE FOR wi == 0 and Vl == 0
  OA = wL ./ (uL + 2*wi);
  idx = isnan(OA);

  % take some derivatives
  alfs_uL =   1./(1 + OA.^2) .* (  -wL ./ ( uL + 2*wi ).^2 );
  alfs_wL =   1./(1 + OA.^2) .* (    1 ./ ( uL + 2*wi ) );
  alfs_wi =   1./(1 + OA.^2) .* (-2*wL ./ ( uL + 2*wi).^2 );

  alfs_uL(idx) = 0;
  alfs_wL(idx) = 0;
  alfs_wi(idx) = 0;


  %% take the derivates with respect to the states
  alfs_x1 = alfs_uL.*uL_x(:,1) + alfs_wL.*wL_x(:,1) + alfs_wi.*wi_x(:,1);
  alfs_x2 = alfs_uL.*uL_x(:,2) + alfs_wL.*wL_x(:,2) + alfs_wi.*wi_x(:,2);
  alfs_x3 = alfs_uL.*uL_x(:,3) + alfs_wL.*wL_x(:,3) + alfs_wi.*wi_x(:,3);
  alfs_x4 = alfs_uL.*uL_x(:,4) + alfs_wL.*wL_x(:,4) + alfs_wi.*wi_x(:,4);
  alfs_x5 = alfs_uL.*uL_x(:,5) + alfs_wL.*wL_x(:,5) + alfs_wi.*wi_x(:,5);
  alfs_x6 = alfs_uL.*uL_x(:,6) + alfs_wL.*wL_x(:,6) + alfs_wi.*wi_x(:,6);

  %% take the derivates with respect to the inputs
  alfs_u1 = alfs_uL.*uL_u(:,1) + alfs_wL.*wL_u(:,1) + alfs_wi.*wi_u(:,1);
  alfs_u2 = alfs_uL.*uL_u(:,2) + alfs_wL.*wL_u(:,2) + alfs_wi.*wi_u(:,2);
  alfs_u3 = alfs_uL.*uL_u(:,3) + alfs_wL.*wL_u(:,3) + alfs_wi.*wi_u(:,3);
  alfs_u4 = alfs_uL.*uL_u(:,4) + alfs_wL.*wL_u(:,4) + alfs_wi.*wi_u(:,4);


  % x = [ u        v        w        p        q        r ]
  y_x = [ alfs_x1  alfs_x2  alfs_x3  alfs_x4  alfs_x5  alfs_x6 ];

  % u = [ T       To      del     i ]
  y_u = [ alfs_u1 alfs_u2 alfs_u3 alfs_u4];

else 
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
