function [y, y_x, y_u] = alf_flaps(x,u,w,ders, alfs,alfs_x,alfs_u)
%
% alf_f = alf_s + tau*del;
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


% control surface deflection 
del = u(:,3);
idx = ~(del==255);

% control surface effectiveness
tau = 0.6;

% angle of attack due to flaps
alff = alfs + tau*del.*idx; 

% outputs
y = alff;

if ders

  % take some derivatives
  alff_alfs = ones(length(del),1);
  alff_del = tau*idx;

  %% take the derivates with respect to the states
  alff_x1 = alff_alfs.*alfs_x(:,1);
  alff_x2 = alff_alfs.*alfs_x(:,2);
  alff_x3 = alff_alfs.*alfs_x(:,3);
  alff_x4 = alff_alfs.*alfs_x(:,4);
  alff_x5 = alff_alfs.*alfs_x(:,5);
  alff_x6 = alff_alfs.*alfs_x(:,6);

  %% take the derivates with respect to the inputs
  alff_u1 = alff_alfs.*alfs_u(:,1);
  alff_u2 = alff_alfs.*alfs_u(:,2);
  alff_u3 = alff_alfs.*alfs_u(:,3) + alff_del;
  alff_u4 = alff_alfs.*alfs_u(:,4);


  % x = [ u        v        w        p        q        r ]
  y_x = [ alff_x1  alff_x2  alff_x3  alff_x4  alff_x5  alff_x6 ];

  % u = [ T       To      del     i ]
  y_u = [ alff_u1 alff_u2 alff_u3 alff_u4 ];

else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end

