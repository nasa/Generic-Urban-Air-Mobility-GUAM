function [F, M, F_x, M_x, F_u, M_u] = poly_aero_wrapper_Mod_du(x,u,rho)

% calling the LpC_model_parameters function. 
blending_method  = 2;
Units.deg = pi/180;
Units.knot = 1.6878;
a = 1125.33; % Speed of sound ft/sec
SimIn.numEngines = 9;
[CModel ] = LpC_model_parameters(SimIn);

% call the aero function at the current state and input location
[X,Y,Z,L,M,N,~] = LpC_aero_p_v2(x,u(5:13),[u(1)+u(2), u(1)-u(2), u(3), u(3), u(4)],rho,a,blending_method,Units,CModel);


% package the forces and moments
F = [X; Y; Z];
M = [L; M; N];

%% Compute the derivatives 

% Numerical derivatives with respect to the states
F_x = zeros(3,6);
M_x = zeros(3,6);
dx = [max(1e-4*x,1e-7)]; % Modified by MJA to align with Matlab step sizes
% dx = [.01; .01; .01; .001; .001; .001]; % Ben's original hardcoded steps
for ii = 1:6
  x1 = x;
  x1(ii) = x(ii)+0.5*dx(ii);
  [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x1, u(5:13), [u(1)+u(2), u(1)-u(2), u(3), u(3), u(4)], ...
      rho, a, blending_method,Units,CModel);

  x2 = x;
  x2(ii) = x(ii)-0.5*dx(ii);
  [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x2, u(5:13), [u(1)+u(2), u(1)-u(2), u(3), u(3), u(4)],...
      rho, a, blending_method,Units,CModel);

  F_x(:,ii) = [(X1-X2)/dx(ii); (Y1-Y2)/dx(ii); (Z1-Z2)/dx(ii)];
  M_x(:,ii) = [(L1-L2)/dx(ii); (M1-M2)/dx(ii); (N1-N2)/dx(ii)];
end

% Numerical derivatives with respect to the inputs
F_u = zeros(3,13);
M_u = zeros(3,13);
du = [max(1e-4*u, 1e-7)]; % Modified by MJA to align with Matlab step sizes
% du = [ repmat(.001,5,1); repmat(.01,9,1)]; % Ben's original hardcoded steps

for ii = 1:13
%     u1 = [u(1)+u(2), u(1)-u(2), u(3), u(3), u(4) u(5:13)]; 
%     u2 = [u(1)+u(2), u(1)-u(2), u(3), u(3), u(4) u(5:13)]; % Set the baseline
% Sign Convention....
    u1 = [u(1)-u(2), u(1)+u(2), u(3), u(3), u(4) u(5:13)]; 
    u2 = [u(1)-u(2), u(1)+u(2), u(3), u(3), u(4) u(5:13)]; % Set the baseline
    switch ii
        case 1 % Flap
            u1(ii) = u(ii)+0.5*du(ii); % left flap
            u1(ii+1) = u(ii+1)+0.5*du(ii); % right flap
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii) = u(ii)-0.5*du(ii);
            u2(ii+1) = u(ii+1)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        case 2 % Aileron
            % Sign convention
            u1(ii-1) = u(ii-1)-0.5*du(ii); % left aileron 
            u1(ii) = u(ii)+0.5*du(ii); % right aileron
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            % Sign convention
            u2(ii-1) = u(ii-1)+0.5*du(ii);
            u2(ii) = u(ii)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        case 3 % Elevator
            u1(ii) = u(ii)+0.5*du(ii); % left elevator
            u1(ii+1) = u(ii+1)+0.5*du(ii); % right elevator
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a,blending_method,Units,CModel);
            
            u2(ii) = u(ii)-0.5*du(ii);
            u2(ii+1) = u(ii+1)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);
        otherwise % Rudder and props...
            u1(ii+1) = u(ii)+0.5*du(ii); % rudder and props
            [X1,Y1,Z1,L1,M1,N1] = LpC_aero_p_v2(x, u1(6:14), u1(1:5), rho ,a, blending_method, Units, CModel);
            
            u2(ii+1) = u(ii)-0.5*du(ii);
            [X2,Y2,Z2,L2,M2,N2] = LpC_aero_p_v2(x, u2(6:14), u2(1:5),rho,a,blending_method,Units,CModel);

    end
    F_u(:,ii) = [(X1-X2)/du(ii); (Y1-Y2)/du(ii); (Z1-Z2)/du(ii)];
    M_u(:,ii) = [(L1-L2)/du(ii); (M1-M2)/du(ii); (N1-N2)/du(ii)];
end


