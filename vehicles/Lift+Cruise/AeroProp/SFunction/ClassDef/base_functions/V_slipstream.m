function [y, y_x, y_u] = V_slipstream(x,u,w,ders,...
                              uL, uL_x, uL_u,...
                              vL, vL_x, vL_u,...
                              wL, wL_x, wL_u,...
                              wi, wi_x, wi_u)
%
% v_s = sqrt( (uL+wi)^2 + vL^2 + wL^2 );
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

% slipstream velocity 
Vs = sqrt( (uL+2*wi).^2 + vL.^2 + wL.^2 );
% outputs
y = Vs;

if ders

  % take some derivatives
  Vs_uL = ( uL+2*wi )./Vs;
  Vs_vL =  vL./Vs;
  Vs_wL =  wL./Vs;
  Vs_wi = 2*(uL + 2*wi)./Vs;

  % handle the divide by zeros
  idx = find(Vs == 0);
  Vs_uL(idx) = 1;
  Vs_vL(idx) = 1;
  Vs_wL(idx) = 1;
  Vs_wi(idx) = 1;
   
  %% take the derivates with respect to the states
  Vs_x1 = Vs_uL.*uL_x(:,1) + Vs_vL.*vL_x(:,1) + Vs_wL.*wL_x(:,1) + Vs_wi.*wi_x(:,1);
  Vs_x2 = Vs_uL.*uL_x(:,2) + Vs_vL.*vL_x(:,2) + Vs_wL.*wL_x(:,2) + Vs_wi.*wi_x(:,2);
  Vs_x3 = Vs_uL.*uL_x(:,3) + Vs_vL.*vL_x(:,3) + Vs_wL.*wL_x(:,3) + Vs_wi.*wi_x(:,3);
  Vs_x4 = Vs_uL.*uL_x(:,4) + Vs_vL.*vL_x(:,4) + Vs_wL.*wL_x(:,4) + Vs_wi.*wi_x(:,4);
  Vs_x5 = Vs_uL.*uL_x(:,5) + Vs_vL.*vL_x(:,5) + Vs_wL.*wL_x(:,5) + Vs_wi.*wi_x(:,5);
  Vs_x6 = Vs_uL.*uL_x(:,6) + Vs_vL.*vL_x(:,6) + Vs_wL.*wL_x(:,6) + Vs_wi.*wi_x(:,6);

  %% take the derivates with respect to the inputs
  Vs_u1 = Vs_uL.*uL_u(:,1) + Vs_vL.*vL_u(:,1) + Vs_wL.*wL_u(:,1) + Vs_wi.*wi_u(:,1);
  Vs_u2 = Vs_uL.*uL_u(:,2) + Vs_vL.*vL_u(:,2) + Vs_wL.*wL_u(:,2) + Vs_wi.*wi_u(:,2);
  Vs_u3 = Vs_uL.*uL_u(:,3) + Vs_vL.*vL_u(:,3) + Vs_wL.*wL_u(:,3) + Vs_wi.*wi_u(:,3);
  Vs_u4 = Vs_uL.*uL_u(:,4) + Vs_vL.*vL_u(:,4) + Vs_wL.*wL_u(:,4) + Vs_wi.*wi_u(:,4);


  % x = [ u      v      w      p      q      r ]
  y_x = [ Vs_x1  Vs_x2  Vs_x3  Vs_x4  Vs_x5  Vs_x6 ];

  % u = [ T     To    del   i ]
  y_u = [ Vs_u1 Vs_u2 Vs_u3 Vs_u4 ];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
