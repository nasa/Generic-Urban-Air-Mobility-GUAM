function [y, y_x, y_u] = L_local(x,u,w,ders,...
                                 cl,cl_x,cl_u,...
                                 cd,cd_x,cd_u,...
                                 alfs,alfs_x,alfs_u,...
                                 alfw,alfw_x,alfw_u,...
                                 qs,qs_x,qs_u)
%
% L = T*sin(alf_w) + To*sin(alf_w) 
%     +( cl_l*cos(alf_w-alf_s) - cd_l*sin(alf_w-alf_s) )*S*qs
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

% thrust properties
T = u(:,1);
A = w(:,4);
To = u(:,2);
Ao = w(:,5);

% wing area
S = w(:,6);

s_alfw = sin(alfw);
c_alfw = cos(alfw);
s_alfw_alfs = sin(alfw-alfs);
c_alfw_alfs = cos(alfw-alfs);

% lift_local
Ll = T.*s_alfw + To.*s_alfw + (cl.*c_alfw_alfs - cd.*s_alfw_alfs).*S.*qs;
% outputs
y = Ll;


if ders 

  % take some derivatives
  Ll_cl =  c_alfw_alfs.*S.*qs;
  Ll_cd = -s_alfw_alfs.*S.*qs;
  Ll_alfw = T.*c_alfw + To.*c_alfw + (-cl.*s_alfw_alfs - cd.*c_alfw_alfs ).*S.*qs;
  Ll_alfs = ( cl.*s_alfw_alfs + cd.*c_alfw_alfs ).*S.*qs;
  Ll_qs =   ( cl.*c_alfw_alfs - cd.*s_alfw_alfs ).*S;
  Ll_T = s_alfw.*(A>0);
  Ll_To = s_alfw.*(Ao>0);


  %% take the derivates with respect to the states
  Ll_x1 = Ll_cl.*cl_x(:,1) + Ll_cd.*cd_x(:,1) + Ll_alfw.*alfw_x(:,1) + Ll_alfs.*alfs_x(:,1) + Ll_qs.*qs_x(:,1);
  Ll_x2 = Ll_cl.*cl_x(:,2) + Ll_cd.*cd_x(:,2) + Ll_alfw.*alfw_x(:,2) + Ll_alfs.*alfs_x(:,2) + Ll_qs.*qs_x(:,2);
  Ll_x3 = Ll_cl.*cl_x(:,3) + Ll_cd.*cd_x(:,3) + Ll_alfw.*alfw_x(:,3) + Ll_alfs.*alfs_x(:,3) + Ll_qs.*qs_x(:,3);
  Ll_x4 = Ll_cl.*cl_x(:,4) + Ll_cd.*cd_x(:,4) + Ll_alfw.*alfw_x(:,4) + Ll_alfs.*alfs_x(:,4) + Ll_qs.*qs_x(:,4);
  Ll_x5 = Ll_cl.*cl_x(:,5) + Ll_cd.*cd_x(:,5) + Ll_alfw.*alfw_x(:,5) + Ll_alfs.*alfs_x(:,5) + Ll_qs.*qs_x(:,5);
  Ll_x6 = Ll_cl.*cl_x(:,6) + Ll_cd.*cd_x(:,6) + Ll_alfw.*alfw_x(:,6) + Ll_alfs.*alfs_x(:,6) + Ll_qs.*qs_x(:,6);

  %% take the derivates with respect to the inputs
  Ll_u1 = Ll_cl.*cl_u(:,1) + Ll_cd.*cd_u(:,1) + Ll_alfw.*alfw_u(:,1) + Ll_alfs.*alfs_u(:,1) + Ll_qs.*qs_u(:,1) + Ll_T;
  Ll_u2 = Ll_cl.*cl_u(:,2) + Ll_cd.*cd_u(:,2) + Ll_alfw.*alfw_u(:,2) + Ll_alfs.*alfs_u(:,2) + Ll_qs.*qs_u(:,2) + Ll_To;
  Ll_u3 = Ll_cl.*cl_u(:,3) + Ll_cd.*cd_u(:,3) + Ll_alfw.*alfw_u(:,3) + Ll_alfs.*alfs_u(:,3) + Ll_qs.*qs_u(:,3);
  Ll_u4 = Ll_cl.*cl_u(:,4) + Ll_cd.*cd_u(:,4) + Ll_alfw.*alfw_u(:,4) + Ll_alfs.*alfs_u(:,4) + Ll_qs.*qs_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ Ll_x1  Ll_x2  Ll_x3  Ll_x4  Ll_x5  Ll_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ Ll_u1  Ll_u2  Ll_u3  Ll_u4];
else
  y_x = [];
  y_u = [];
end
