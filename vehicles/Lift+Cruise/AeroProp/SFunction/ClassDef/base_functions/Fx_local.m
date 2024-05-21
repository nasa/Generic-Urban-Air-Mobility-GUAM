function [y, y_x, y_u] = Fx_local(x,u,w,ders,...
                             cl,cl_x,cl_u,...
                             cd,cd_x,cd_u,...
                             alfs,alfs_x,alfs_u,...
                             qs,qs_x,qs_u)
%
% Fx = (T*cos(i_w) + To*cos(i_w) - (cl_l*sin(i_w-alf_s) + cd_l*cos(i_w-alf_s) )*S*qs)
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

% thrusts
T = u(:,1);
To = u(:,2);
A = w(:,4);
Ao = w(:,5);

% tilt angle 
iw = u(:,4);

% wing area
S = w(:,6);

c_iw = cos(iw);
s_iw = sin(iw);
c_iw_alfs = cos(iw-alfs);
s_iw_alfs = sin(iw-alfs);

% axial Force
Fx = T.*c_iw + To.*c_iw - (cl.*s_iw_alfs + cd.*c_iw_alfs).*S.*qs;

% outputs
y = Fx;

if ders

  % take some derivatives
  Fx_cl = -s_iw_alfs.*S.*qs;
  Fx_cd = -c_iw_alfs.*S.*qs;
  Fx_iw = -T.*s_iw - To.*s_iw - ( cl.*c_iw_alfs - cd.*s_iw_alfs ).*S.*qs;
  Fx_alfs = ( cl.*c_iw_alfs - cd.*s_iw_alfs ).*S.*qs;
  Fx_qs =  -( cl.*s_iw_alfs + cd.*c_iw_alfs ).*S;
  Fx_T  = c_iw.*(A>0);
  Fx_To = c_iw.*(Ao>0);

  %% take the derivates with respect to the states
  Fx_x1 = Fx_cl.*cl_x(:,1) + Fx_cd.*cd_x(:,1) + Fx_alfs.*alfs_x(:,1) + Fx_qs.*qs_x(:,1);
  Fx_x2 = Fx_cl.*cl_x(:,2) + Fx_cd.*cd_x(:,2) + Fx_alfs.*alfs_x(:,2) + Fx_qs.*qs_x(:,2);
  Fx_x3 = Fx_cl.*cl_x(:,3) + Fx_cd.*cd_x(:,3) + Fx_alfs.*alfs_x(:,3) + Fx_qs.*qs_x(:,3);
  Fx_x4 = Fx_cl.*cl_x(:,4) + Fx_cd.*cd_x(:,4) + Fx_alfs.*alfs_x(:,4) + Fx_qs.*qs_x(:,4);
  Fx_x5 = Fx_cl.*cl_x(:,5) + Fx_cd.*cd_x(:,5) + Fx_alfs.*alfs_x(:,5) + Fx_qs.*qs_x(:,5);
  Fx_x6 = Fx_cl.*cl_x(:,6) + Fx_cd.*cd_x(:,6) + Fx_alfs.*alfs_x(:,6) + Fx_qs.*qs_x(:,6);

  %% take the derivates with respect to the inputs
  Fx_u1 = Fx_cl.*cl_u(:,1) + Fx_cd.*cd_u(:,1) + Fx_alfs.*alfs_u(:,1) + Fx_qs.*qs_u(:,1) + Fx_T;
  Fx_u2 = Fx_cl.*cl_u(:,2) + Fx_cd.*cd_u(:,2) + Fx_alfs.*alfs_u(:,2) + Fx_qs.*qs_u(:,2) + Fx_To;
  Fx_u3 = Fx_cl.*cl_u(:,3) + Fx_cd.*cd_u(:,3) + Fx_alfs.*alfs_u(:,3) + Fx_qs.*qs_u(:,3);
  Fx_u4 = Fx_cl.*cl_u(:,4) + Fx_cd.*cd_u(:,4) + Fx_alfs.*alfs_u(:,4) + Fx_qs.*qs_u(:,4) + Fx_iw;

  % x = [ u      v      w      p      q      r ]
  y_x = [ Fx_x1  Fx_x2  Fx_x3  Fx_x4  Fx_x5  Fx_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ Fx_u1  Fx_u2  Fx_u3  Fx_u4];
else
  [N,~]=size(x);
  y_x = zeros(N,6);
  y_u = zeros(N,4);
end
