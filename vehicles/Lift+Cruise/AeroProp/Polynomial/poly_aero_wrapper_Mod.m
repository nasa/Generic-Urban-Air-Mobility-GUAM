function [F, M, F_x, M_x, F_u, M_u] = poly_aero_wrapper_Mod(x,u,rho,XRAW, kts2ft, a, blending_method)

% calling the LpC_model_parameters function. 
% blending_method  = 2;
Units.deg = pi/180;
Units.knot = kts2ft;
SimIn.numEngines = 9;
% [~, CModel ] = LpC_model_parameters(SimIn);
[CModel ] = LpC_model_parameters(SimIn);

% call the aero function at the current state and input location
[X,Y,Z,L,M,N,~] = LpC_aero_p_v2(x,u(6:14),u(1:5),rho,a,blending_method,Units,CModel);


% package the forces and moments
F = [X; Y; Z];
M = [L; M; N];

%% Compute the derivatives 

% Numerical derivatives with respect to the states
% XRAW = [theta phi p q r UH WH];
% Rot = Rx(phi)*Ry(theta);
% S = S_matrix(phi,theta);
% 
% % state vectors
% om = [p; q; r];
% vb = Rot*[UH; 0; WH];

F_x = zeros(3,5);
M_x = zeros(3,5);
% This is default from matlab >> nuderst cmd which is default step size for numerical differention
dx = [max(1e-4*abs(XRAW(1:2)),1e-7) max(1e-4*abs(x(1:3)),1e-7)]; % theta, phi, p, q, r
% dx = [max(1e-4*abs(XRAW(1:2)),1e-7).01; .01; .001; .001; .001]; % theta, phi, p, q, r
for ii = 1:5
  switch ii
    case 1
        x1 = [Rx(XRAW(2))*Ry(XRAW(1)+0.5*dx(ii))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)];
        x2 = [Rx(XRAW(2))*Ry(XRAW(1)-0.5*dx(ii))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)];
    case 2
        x1 = [Rx(XRAW(2)+0.5*dx(ii))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)];
        x2 = [Rx(XRAW(2)-0.5*dx(ii))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)];
    case 3
        x1 = [Rx(XRAW(2))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3)+0.5*dx(ii);XRAW(4);XRAW(5)];
        x2 = [Rx(XRAW(2)-0.5*dx(ii))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3)-0.5*dx(ii);XRAW(4);XRAW(5)];
    case 4
        x1 = [Rx(XRAW(2))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4)+0.5*dx(ii);XRAW(5)];
        x2 = [Rx(XRAW(2)-0.5*dx(ii))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4)-0.5*dx(ii);XRAW(5)];
    case 5
        x1 = [Rx(XRAW(2))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)+0.5*dx(ii)];
        x2 = [Rx(XRAW(2)-0.5*dx(ii))*Ry(XRAW(1))*[XRAW(6); 0; XRAW(7)]; XRAW(3);XRAW(4);XRAW(5)-0.5*dx(ii)];
  end
  x1 = x;
  x1(ii) = x(ii)+0.5*dx(ii);
  [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x1, u(6:14), u(1:5),rho,a,blending_method,Units,CModel);

  x2 = x;
  x2(ii) = x(ii)-0.5*dx(ii);
  [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x2, u(6:14), u(1:5), rho,a,blending_method,Units,CModel);

  F_x(:,ii) = [(X1-X2)/dx(ii); (Y1-Y2)/dx(ii); (Z1-Z2)/dx(ii)];
  M_x(:,ii) = [(L1-L2)/dx(ii); (M1-M2)/dx(ii); (N1-N2)/dx(ii)];
end

% Numerical derivatives with respect to the inputs
F_u = zeros(3,13);
M_u = zeros(3,13);
% This is default from matlab >> nuderst cmd which is default step size for numerical differention
du = max(1e-4*abs(u),1e-7); % df, da, de, dr, drotors
%[ repmat(.001,4,1); repmat(.01,9,1)]; % df, da, de, dr, drotors

for ii = 1:13
    u1 = u;
    u2 = u; % Set the baseline
    switch ii
        case 1 % Flap
            u1(ii) = u(ii)+0.5*du(ii); % left flap
            u1(ii+1) = u(ii+1)+0.5*du(ii); % right flap
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii) = u(ii)-0.5*du(ii);
            u2(ii+1) = u(ii+1)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        case 2 % Aileron
            u1(ii-1) = u(ii-1)+0.5*du(ii); % left aileron
            u1(ii) = u(ii)-0.5*du(ii); % right aileron
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii-1) = u(ii-1)-0.5*du(ii);
            u2(ii) = u(ii)+0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        case 3 % Elevator
            u1(ii) = u(ii)+0.5*du(ii); % left elevator
            u1(ii+1) = u(ii+1)+0.5*du(ii); % right elevator
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii) = u(ii)-0.5*du(ii);
            u2(ii+1) = u(ii+1)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        otherwise % Rudder and props...
            u1(ii+1) = u(ii+1)+0.5*du(ii); % rudder and props
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii+1) = u(ii+1)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);

    end
    F_u(:,ii) = [(X1-X2)/du(ii); (Y1-Y2)/du(ii); (Z1-Z2)/du(ii)];
    M_u(:,ii) = [(L1-L2)/du(ii); (M1-M2)/du(ii); (N1-N2)/du(ii)];
end


