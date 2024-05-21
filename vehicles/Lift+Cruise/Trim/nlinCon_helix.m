function [ c, ceq, gradc, gradceq ] = nlinCon_helix(x, TRIM_POINT, LPC, GRAV, RHO, Ns,  Np, FreeVars, gang_vec, kts2ft, a, blending_method)

global POLY
% gravity vector
g = [0; 0; GRAV];

% Trim point
UH    = TRIM_POINT(1);
WH    = TRIM_POINT(2);
R     = TRIM_POINT(3);
dPSI  = UH/R;

X_IDX = 1;
TP_IDX = 4;

% pitch angle 
if FreeVars(1)
  theta = x(X_IDX);
  X_IDX = X_IDX+1;
else 
  if gang_vec(1)~=0
    theta   = x(sum(FreeVars(1:find(gang_vec == gang_vec(1),1,'first'))));
    TP_IDX  = TP_IDX+1;
  else
      theta = TRIM_POINT(TP_IDX);
      TP_IDX = TP_IDX+1;
  end
end

% roll angle 
if FreeVars(2)
  phi = x(X_IDX);
  X_IDX = X_IDX+1;
else 
  if gang_vec(2)~=0
    phi     = x(sum(FreeVars(1:find(gang_vec == gang_vec(2),1,'first'))));
    TP_IDX  = TP_IDX+1;
  else
    phi = TRIM_POINT(TP_IDX);
    TP_IDX = TP_IDX+1;
  end
end

% roll rate p
if FreeVars(3)
  p = x(X_IDX);
  X_IDX = X_IDX+1;
else 
  if gang_vec(3)~=0
    p       = x(sum(FreeVars(1:find(gang_vec == gang_vec(3),1,'first'))));
    TP_IDX  = TP_IDX+1;
  else
    p = TRIM_POINT(TP_IDX);
    TP_IDX = TP_IDX+1;
  end
end

% pitch rate q
if FreeVars(4)
  q = x(X_IDX);
  X_IDX = X_IDX+1;
else 
  if gang_vec(4)~=0
    q       = x(sum(FreeVars(1:find(gang_vec == gang_vec(4),1,'first'))));
    TP_IDX  = TP_IDX+1;
  else
      q = TRIM_POINT(TP_IDX);
      TP_IDX = TP_IDX+1;
  end
end

% yaw rate r
if FreeVars(5)
  r = x(X_IDX);
  X_IDX = X_IDX+1;
else
  if gang_vec(5)~=0
    r       = x(sum(FreeVars(1:find(gang_vec == gang_vec(5),1,'first'))));
    TP_IDX  = TP_IDX+1;
  else
    r = TRIM_POINT(TP_IDX);
    TP_IDX = TP_IDX+1;
  end
end

% Control Surfaces
for ii = 1:Ns
  if FreeVars(ii+5)
    del(ii) = x(X_IDX);
    X_IDX = X_IDX+1;
  else
    if gang_vec(ii+5)~=0
      del(ii) = x(sum(FreeVars(1:find(gang_vec == gang_vec(ii+5),1,'first'))));
      TP_IDX  = TP_IDX+1;
    else
      del(ii) = TRIM_POINT(TP_IDX);
      TP_IDX = TP_IDX+1;
    end
  end
end

% Propellers and Rotors
for ii = 1:Np
  if FreeVars(ii+Ns+5)
    prop(ii) = x(X_IDX);
    X_IDX = X_IDX+1;
  else
    if gang_vec(ii+Ns+5)~=0
      prop(ii) = x(sum(FreeVars(1:find(gang_vec == gang_vec(ii+Ns+5),1,'first'))));
      TP_IDX  = TP_IDX+1;
    else
      prop(ii) = TRIM_POINT(TP_IDX);
      TP_IDX = TP_IDX+1;
    end
  end
end

% get the rotation matrices 
Rot = Rx(phi)*Ry(theta);
S = S_matrix(phi,theta);

% state vectors
om = [p; q; r];
vb = Rot*[UH; 0; WH];

if POLY 

  X = [vb' om'];
  N = prop;
  XRAW = [theta phi p q r UH WH];
  %N = [repmat(prop(1),1,4)  repmat(prop(2),1,4)  prop(3)];
  %D = [del(1) -del(1) del(2) del(2) del(3)];
  
  % Aileron sign convention here....
  %D = [del(1)+del(2) del(1)-del(2) del(3) del(3) del(4)];
  D = [del(1)-del(2) del(1)+del(2) del(3) del(3) del(4)]; % Match what is in the simulation....

  U = [D, N];
  if nargout >2 
    [F, M, F_x, M_x, F_u, M_u] = poly_aero_wrapper_Mod(X, U, RHO, XRAW, kts2ft, a, blending_method); 
  else 
    [F, M] = poly_aero_wrapper_Mod(X,U,RHO, XRAW, kts2ft, a, blending_method);
  end

else

  % set the control inputs
  LPC.om_p(1:4) = prop(1:4);
  LPC.om_p(5:8) = prop(5:8);
  LPC.om_p(9)   = prop(9);
  % LPC.om_p(1:4) = prop(1);
  % LPC.om_p(5:8) = prop(2);
  % LPC.om_p(9)   = prop(3);
  LPC.del_f     = del(1);
  LPC.del_a     = del(2);
  LPC.del_e     = del(3);
  LPC.del_r     = del(4);

  % calulate aircraft aero
  if nargout >2
    [F M] = LPC.aero(RHO, vb, om, 0);
  else
      [F M] = LPC.aero(RHO, vb, om, 0);
  end

end
%fprintf('[%f %f %f]\n', F(1), F(2), F(3));
m = LPC.mass;
J = LPC.I;
iJ = inv(J);

% inequality constraints
%  c(x) <= 0
c = [];

% equality constraints
d_eta  =  S*om;
d_vb   = -cross(om,vb) + Rot*g + 1/m*F;
d_om   =  iJ*(-cross(om,J*om) + M);
ceq = [d_eta; d_vb; d_om] - [0; 0; dPSI; 0; 0; 0; 0; 0; 0];

if nargout > 2 % gradient required
    % Specify inequality gradient
    gradc = []; % No inequalities, so gradient is empty

    % Specify equality gradient
    %  ***************** ceq = [d_eta; d_vb; d_om] ************************
    % d_eta  =  S*om;
    % d_vb   = -cross(om,vb) + Rot*g + 1/m*F;
    % d_om   =  iJ*(-cross(om,J*om) + M);

    % get the rotation matrices 
    % Rot = Rx(phi)*Ry(theta);
    % S = S_matrix(phi,theta);

    % state vectors
    % om = [p; q; r];
    % vb = Rot*[UH; 0; WH];

    % Placeholder for gradceq
    gradceq = [ ];

    % Compute gradceq_eta
    gradceq_eta_theta       = [sin(phi)/(cos(theta))^2*q+cos(phi)/(cos(theta)^2)*r;0;-sin(phi)*sin(theta)*q/(cos(theta)^2)-cos(phi)*sin(theta)*r/(cos(theta)^2)];
    gradceq_eta_phi         = [cos(phi)*tan(theta)*q-sin(phi)*tan(theta)*r;-sin(phi)*q-cos(phi)*r; cos(phi)*q/cos(theta)-sin(phi)*r/cos(theta)];
    gradceq_eta_pqr         = S;
    gradceq_eta_eff         = zeros(3,Ns+Np);
    gradceq_eta             = [gradceq_eta_theta gradceq_eta_phi gradceq_eta_pqr gradceq_eta_eff];
    gradceq_eta_FreeVars    = gradceq_eta(:,FreeVars);

    % Compute gradceq_vb
    Rx_phi              = Rx(phi);
    dRx_phi             = [0 0 0; 0 -sin(phi) cos(phi); 0 -cos(phi) -sin(phi)];
    Ry_theta            = Ry(theta);
    dRy_theta           = [-sin(theta) 0 -cos(theta); 0 0 0; cos(theta) 0 -sin(theta)];
    cross_om            = [0 -r -q;r 0 -p; -q p 0];
    d_cross_om_p        = [0 0 0; 0 0 -1; 0 1 0];
    d_cross_om_q        = [0 0 1; 0 0 0; -1 0 0];
    d_cross_om_r        = [0 -1 0; 1 0 0; 0 0 0];
    gradceq_vb_theta    = -cross_om*Rx_phi*dRy_theta*[UH; 0; WH]+Rx_phi*dRy_theta(:,3)*GRAV+1/m*F_x(:,1);
    gradceq_vb_phi      = -cross_om*dRx_phi*Ry_theta*[UH;0;WH]+dRx_phi*Ry_theta(:,3)*GRAV+1/m*F_x(:,2);
    gradceq_vb_p        = d_cross_om_p*vb+1/m*F_x(:,3);
    gradceq_vb_q        = d_cross_om_q*vb+1/m*F_x(:,4);
    gradceq_vb_r        = d_cross_om_r*vb+1/m*F_x(:,5);
    gradceq_vb_eff      = 1/m*F_u;
    gradceq_vb          = [gradceq_vb_theta gradceq_vb_phi gradceq_vb_p gradceq_vb_q gradceq_vb_r gradceq_vb_eff];
    gradceq_vb_FreeVars = gradceq_vb(:,FreeVars);

    % Compute gradceq_om
    gradceq_om_theta    = iJ*M_x(:,1);
    gradceq_om_phi      = iJ*M_x(:,2);
    gradceq_om_p        = iJ*M_x(:,3)-iJ*d_cross_om_p*J*om-iJ*cross_om*J(:,1);
    gradceq_om_q        = iJ*M_x(:,4)-iJ*d_cross_om_q*J*om-iJ*cross_om*J(:,2);
    gradceq_om_r        = iJ*M_x(:,5)-iJ*d_cross_om_r*J*om-iJ*cross_om*J(:,3);
    gradceq_om_eff      = iJ*M_u;
    gradceq_om          = [gradceq_om_theta gradceq_om_phi gradceq_om_p gradceq_om_q gradceq_om_r gradceq_om_eff];
    gradceq_om_FreeVars = gradceq_om(:, FreeVars);

    % Compile into one vector
    gradceq = [gradceq_eta_FreeVars; gradceq_vb_FreeVars; gradceq_om_FreeVars]';
    disp('')
end