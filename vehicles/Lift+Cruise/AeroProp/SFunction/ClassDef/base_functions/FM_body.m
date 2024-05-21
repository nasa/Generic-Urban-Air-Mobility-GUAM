function [FM, FM_x, FM_u] = FM_body(x,u,w,aero_coefs_pp,ders) %#codegen
%
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


gam = w(:,8);

% y location of strip
bx = w(:,1);
by = w(:,2);
bz = w(:,3);

% local flow conditions 
[uL, uL_x, uL_u] = u_local(x,u,w,ders);
[vL, vL_x, vL_u] = v_local(x,u,w,ders);
[wL, wL_x, wL_u] = w_local(x,u,w,ders);

% propeller induced velocity
[wi, wi_x, wi_u] = w_induced(x,u,w,ders,uL,uL_x,uL_u);

% wing angle of attack 
[alfs, alfs_x, alfs_u] = alf_slipstream(x,u,w,ders,...
                                      uL,uL_x,uL_u,...
                                      wL,wL_x,wL_u,...
                                      wi,wi_x,wi_u);

% slipstream angle of attack including flap deflection contribution
[alff, alff_x, alff_u] = alf_flaps(x,u,w,ders,alfs,alfs_x,alfs_u);

% local lift coefficient
[cl, cl_x, cl_u] = cl_local(x,u,w,aero_coefs_pp,ders,...
                          alff,alff_x,alff_u);

% local drag coefficient
[cd, cd_x, cd_u] = cd_local(x,u,w,aero_coefs_pp,ders,...
                          alff,alff_x,alff_u);

% local moment coefficient
[cm, cm_x, cm_u] = cm_local(x,u,w,aero_coefs_pp,ders,...
                          alff,alff_x,alff_u);

% local flow conditions 
[Vs, Vs_x, Vs_u] = V_slipstream(x,u,w,ders,...
                              uL,uL_x,uL_u,...
                              vL,vL_x,vL_u,...
                              wL,wL_x,wL_u,...
                              wi,wi_x,wi_u);

% slipstream dynamic pressure
[qs, qs_x, qs_u] = q_slipstream(x,u,w,ders,Vs,Vs_x,Vs_u);

% get the local Fx
[Fxl, Fxl_x, Fxl_u] = Fx_local(x,u,w,ders,...
                             cl,cl_x,cl_u,...
                             cd,cd_x,cd_u,...
                             alfs,alfs_x,alfs_u,...
                             qs,qs_x,qs_u);

% get the local Fz
[Fzl, Fzl_x, Fzl_u] = Fz_local(x,u,w,ders,...
                             cl,cl_x,cl_u,...
                             cd,cd_x,cd_u,...
                             alfs,alfs_x,alfs_u,...
                             qs,qs_x,qs_u);

% pitching moment of wing
[Myl, Myl_x, Myl_u] = My_local(x,u,w,ders,...
                             cm,cm_x,cm_u,...
                             qs,qs_x,qs_u);

% total forces and moments
Fx = Fxl;
Fy = -Fzl.*sin(gam);
Fz = Fzl.*cos(gam);
Mx = Fz.*by - Fy.*bz;
My = Myl.*cos(gam) - Fz.*bx + Fx.*bz;
Mz = Myl.*sin(gam) - Fx.*by + Fy.*bx;

FM = [Fx Fy Fz Mx My Mz];

if ders

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% FX
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % allocate storage
  [m,n] = size(Fxl_x);
  Fx_x = zeros(m,n);
  
  %% take the derivates with respect to the states
  Fx_x(:,1) = Fxl_x(:,1);
  Fx_x(:,2) = Fxl_x(:,2);
  Fx_x(:,3) = Fxl_x(:,3);
  Fx_x(:,4) = Fxl_x(:,4);
  Fx_x(:,5) = Fxl_x(:,5);
  Fx_x(:,6) = Fxl_x(:,6);

  % allocate storage
  [m,n] = size(Fxl_u);
  Fx_u = zeros(m,n);

  %% take the derivates with respect to the inputs
  Fx_u(:,1) = Fxl_u(:,1);
  Fx_u(:,2) = Fxl_u(:,2);
  Fx_u(:,3) = Fxl_u(:,3);
  Fx_u(:,4) = Fxl_u(:,4);


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% FY
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  Fy_Fzl = -sin(gam);

  % allocate storage
  [m,n] = size(Fzl_x);
  Fy_x = zeros(m,n);

  % take the derivates with respect to the states
  Fy_x(:,1) = Fy_Fzl.*Fzl_x(:,1);
  Fy_x(:,2) = Fy_Fzl.*Fzl_x(:,2);
  Fy_x(:,3) = Fy_Fzl.*Fzl_x(:,3);
  Fy_x(:,4) = Fy_Fzl.*Fzl_x(:,4);
  Fy_x(:,5) = Fy_Fzl.*Fzl_x(:,5);
  Fy_x(:,6) = Fy_Fzl.*Fzl_x(:,6);

  % allocate storage
  [m,n] = size(Fzl_u);
  Fy_u = zeros(m,n);
  
  % take the derivates with respect to the inputs
  Fy_u(:,1) = Fy_Fzl.*Fzl_u(:,1);
  Fy_u(:,2) = Fy_Fzl.*Fzl_u(:,2);
  Fy_u(:,3) = Fy_Fzl.*Fzl_u(:,3);
  Fy_u(:,4) = Fy_Fzl.*Fzl_u(:,4);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% FZ
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  Fz_Fzl = cos(gam);

  % allocate storage
  [m,n] = size(Fzl_x);
  Fz_x = zeros(m,n);

  % take the derivates with respect to the states
  Fz_x(:,1) = Fz_Fzl.*Fzl_x(:,1);
  Fz_x(:,2) = Fz_Fzl.*Fzl_x(:,2);
  Fz_x(:,3) = Fz_Fzl.*Fzl_x(:,3);
  Fz_x(:,4) = Fz_Fzl.*Fzl_x(:,4);
  Fz_x(:,5) = Fz_Fzl.*Fzl_x(:,5);
  Fz_x(:,6) = Fz_Fzl.*Fzl_x(:,6);

  % allocate storage
  [m,n] = size(Fzl_u);
  Fz_u = zeros(m,n);

  % take the derivates with respect to the inputs
  Fz_u(:,1) = Fz_Fzl.*Fzl_u(:,1);
  Fz_u(:,2) = Fz_Fzl.*Fzl_u(:,2);
  Fz_u(:,3) = Fz_Fzl.*Fzl_u(:,3);
  Fz_u(:,4) = Fz_Fzl.*Fzl_u(:,4);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% MX 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % take some derivatives
  Mx_Fz =  by;
  Mx_Fy = -bz;

  % allocate storage
  [m,n] = size(Fz_x);
  Mx_x = zeros(m,n);

  % take the derivates with respect to the states
  Mx_x(:,1) = Mx_Fz.*Fz_x(:,1) + Mx_Fy.*Fy_x(:,1);
  Mx_x(:,2) = Mx_Fz.*Fz_x(:,2) + Mx_Fy.*Fy_x(:,2);
  Mx_x(:,3) = Mx_Fz.*Fz_x(:,3) + Mx_Fy.*Fy_x(:,3);
  Mx_x(:,4) = Mx_Fz.*Fz_x(:,4) + Mx_Fy.*Fy_x(:,4);
  Mx_x(:,5) = Mx_Fz.*Fz_x(:,5) + Mx_Fy.*Fy_x(:,5);
  Mx_x(:,6) = Mx_Fz.*Fz_x(:,6) + Mx_Fy.*Fy_x(:,6);

  % allocate storage
  [m,n] = size(Fz_u);
  Mx_u = zeros(m,n);

  % take the derivates with respect to the inputs
  Mx_u(:,1) = Mx_Fz.*Fz_u(:,1) + Mx_Fy.*Fy_u(:,1); 
  Mx_u(:,2) = Mx_Fz.*Fz_u(:,2) + Mx_Fy.*Fy_u(:,2);
  Mx_u(:,3) = Mx_Fz.*Fz_u(:,3) + Mx_Fy.*Fy_u(:,3);
  Mx_u(:,4) = Mx_Fz.*Fz_u(:,4) + Mx_Fy.*Fy_u(:,4);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% MY
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % take some derivatives
  My_Myl = cos(gam);
  My_Fz = -bx;
  My_Fx =  bz;

  % allocate storage
  [m,n] = size(Myl_x);
  My_x = zeros(m,n);

  % take the derivates with respect to the states
  My_x(:,1) = My_Myl.*Myl_x(:,1) + My_Fz.*Fz_x(:,1) + My_Fx.*Fx_x(:,1);
  My_x(:,2) = My_Myl.*Myl_x(:,2) + My_Fz.*Fz_x(:,2) + My_Fx.*Fx_x(:,2);
  My_x(:,3) = My_Myl.*Myl_x(:,3) + My_Fz.*Fz_x(:,3) + My_Fx.*Fx_x(:,3);
  My_x(:,4) = My_Myl.*Myl_x(:,4) + My_Fz.*Fz_x(:,4) + My_Fx.*Fx_x(:,4);
  My_x(:,5) = My_Myl.*Myl_x(:,5) + My_Fz.*Fz_x(:,5) + My_Fx.*Fx_x(:,5);
  My_x(:,6) = My_Myl.*Myl_x(:,6) + My_Fz.*Fz_x(:,6) + My_Fx.*Fx_x(:,6);

  % allocate storage
  [m,n] = size(Myl_u);
  My_u = zeros(m,n);

  % take the derivates with respect to the inputs
  My_u(:,1) = My_Myl.*Myl_u(:,1) + My_Fz.*Fz_u(:,1) + My_Fx.*Fx_u(:,1); 
  My_u(:,2) = My_Myl.*Myl_u(:,2) + My_Fz.*Fz_u(:,2) + My_Fx.*Fx_u(:,2);
  My_u(:,3) = My_Myl.*Myl_u(:,3) + My_Fz.*Fz_u(:,3) + My_Fx.*Fx_u(:,3);
  My_u(:,4) = My_Myl.*Myl_u(:,4) + My_Fz.*Fz_u(:,4) + My_Fx.*Fx_u(:,4);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% MZ
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % take some derivatives
  Mz_Myl = sin(gam);
  Mz_Fx = -by;
  Mz_Fy =  bx;

  % allocate storage
  [m,n] = size(Myl_x);
  Mz_x = zeros(m,n);

  % take the derivates with respect to the states
  Mz_x(:,1) = Mz_Myl.*Myl_x(:,1) + Mz_Fx.*Fx_x(:,1) + Mz_Fy.*Fy_x(:,1);
  Mz_x(:,2) = Mz_Myl.*Myl_x(:,2) + Mz_Fx.*Fx_x(:,2) + Mz_Fy.*Fy_x(:,2);
  Mz_x(:,3) = Mz_Myl.*Myl_x(:,3) + Mz_Fx.*Fx_x(:,3) + Mz_Fy.*Fy_x(:,3);
  Mz_x(:,4) = Mz_Myl.*Myl_x(:,4) + Mz_Fx.*Fx_x(:,4) + Mz_Fy.*Fy_x(:,4);
  Mz_x(:,5) = Mz_Myl.*Myl_x(:,5) + Mz_Fx.*Fx_x(:,5) + Mz_Fy.*Fy_x(:,5);
  Mz_x(:,6) = Mz_Myl.*Myl_x(:,6) + Mz_Fx.*Fx_x(:,6) + Mz_Fy.*Fy_x(:,6);

  % allocate storage
  [m,n] = size(Myl_u);
  Mz_u = zeros(m,n);

  % take the derivates with respect to the inputs
  Mz_u(:,1) = Mz_Myl.*Myl_u(:,1) + Mz_Fx.*Fx_u(:,1) + Mz_Fy.*Fy_u(:,1);
  Mz_u(:,2) = Mz_Myl.*Myl_u(:,2) + Mz_Fx.*Fx_u(:,2) + Mz_Fy.*Fy_u(:,2);
  Mz_u(:,3) = Mz_Myl.*Myl_u(:,3) + Mz_Fx.*Fx_u(:,3) + Mz_Fy.*Fy_u(:,3);
  Mz_u(:,4) = Mz_Myl.*Myl_u(:,4) + Mz_Fx.*Fx_u(:,4) + Mz_Fy.*Fy_u(:,4);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Package for shipping
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % allocate storage
  [m,n] = size(Fx_x);
  FM_x = zeros(m,n,6);

  FM_x(:,:,1) = Fx_x;
  FM_x(:,:,2) = Fy_x;
  FM_x(:,:,3) = Fz_x;
  FM_x(:,:,4) = Mx_x;
  FM_x(:,:,5) = My_x;
  FM_x(:,:,6) = Mz_x;

  % allocate storage
  [m,n] = size(Fx_u);
  FM_u = zeros(m,n,6);

  FM_u(:,:,1) = Fx_u;
  FM_u(:,:,2) = Fy_u;
  FM_u(:,:,3) = Fz_u;
  FM_u(:,:,4) = Mx_u;
  FM_u(:,:,5) = My_u;
  FM_u(:,:,6) = Mz_u;

else 
  [N,~]=size(x);
  FM_x = zeros(N,6);
  FM_u = zeros(N,4);
end



