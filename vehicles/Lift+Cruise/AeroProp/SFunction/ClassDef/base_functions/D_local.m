function [y, y_x, y_u] = D_local(x,u,w,ders...
                         cl,cl_x,cl_u,...
                         cd,cd_x,cd_u,...
                         alfs,alfs_x,alfs_u,...
                         alfw,alfw_x,alfw_u,...
                         qs,qs_x,qs_u)
%
% D = -T*cos(alfw) - To*cos(alfw) 
%     + ( cl_l*sin(alf_w-alf_s) + cd_l*cos(alf_w-alf_s) )*S*qs
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

% drag
Dl = -T.*c_alfw -To.*c_alfw + (cl.*s_alfw_alfs + cd.*c_alfw_alfs).*S.*qs;

% outputs
y = Dl;

if ders

  % take some derivatives
  Dl_cl =  s_alfw_alfs.*S.*qs;
  Dl_cd =  c_alfw_alfs.*S.*qs;
  Dl_alfw = T.*s_alfw + To.*s_alfw + ( cl.*c_alfw_alfs - cd.*s_alfw_alfs ).*S.*qs;
  Dl_alfs = (-cl.*c_alfw_alfs + cd.*s_alfw_alfs ).*S.*qs;
  Dl_qs =   ( cl.*s_alfw_alfs + cd.*c_alfw_alfs ).*S;
  Dl_T = -c_alfw.*(A>0);
  Dl_To = -c_alfw.*(Ao>0);


  %% take the derivates with respect to the states
  Dl_x1 = Dl_cl.*cl_x(:,1) + Dl_cd.*cd_x(:,1) + Dl_alfw.*alfw_x(:,1) + Dl_alfs.*alfs_x(:,1) + Dl_qs.*qs_x(:,1);
  Dl_x2 = Dl_cl.*cl_x(:,2) + Dl_cd.*cd_x(:,2) + Dl_alfw.*alfw_x(:,2) + Dl_alfs.*alfs_x(:,2) + Dl_qs.*qs_x(:,2);
  Dl_x3 = Dl_cl.*cl_x(:,3) + Dl_cd.*cd_x(:,3) + Dl_alfw.*alfw_x(:,3) + Dl_alfs.*alfs_x(:,3) + Dl_qs.*qs_x(:,3);
  Dl_x4 = Dl_cl.*cl_x(:,4) + Dl_cd.*cd_x(:,4) + Dl_alfw.*alfw_x(:,4) + Dl_alfs.*alfs_x(:,4) + Dl_qs.*qs_x(:,4);
  Dl_x5 = Dl_cl.*cl_x(:,5) + Dl_cd.*cd_x(:,5) + Dl_alfw.*alfw_x(:,5) + Dl_alfs.*alfs_x(:,5) + Dl_qs.*qs_x(:,5);
  Dl_x6 = Dl_cl.*cl_x(:,6) + Dl_cd.*cd_x(:,6) + Dl_alfw.*alfw_x(:,6) + Dl_alfs.*alfs_x(:,6) + Dl_qs.*qs_x(:,6);

  %% take the derivates with respect to the inputs
  Dl_u1 = Dl_cl.*cl_u(:,1) + Dl_cd.*cd_u(:,1) + Dl_alfw.*alfw_u(:,1) + Dl_alfs.*alfs_u(:,1) + Dl_qs.*qs_u(:,1) + Dl_T;
  Dl_u2 = Dl_cl.*cl_u(:,2) + Dl_cd.*cd_u(:,2) + Dl_alfw.*alfw_u(:,2) + Dl_alfs.*alfs_u(:,2) + Dl_qs.*qs_u(:,2) + Dl_To;
  Dl_u3 = Dl_cl.*cl_u(:,3) + Dl_cd.*cd_u(:,3) + Dl_alfw.*alfw_u(:,3) + Dl_alfs.*alfs_u(:,3) + Dl_qs.*qs_u(:,3);
  Dl_u4 = Dl_cl.*cl_u(:,4) + Dl_cd.*cd_u(:,4) + Dl_alfw.*alfw_u(:,4) + Dl_alfs.*alfs_u(:,4) + Dl_qs.*qs_u(:,4);

  % x = [ u      v      w      p      q      r ]
  y_x = [ Dl_x1  Dl_x2  Dl_x3  Dl_x4  Dl_x5  Dl_x6 ];

  % u = [ T      To     del    i ]
  y_u = [ Dl_u1  Dl_u2  Dl_u3  Dl_u4];
else
  y_x = [];
  y_u = [];
end
