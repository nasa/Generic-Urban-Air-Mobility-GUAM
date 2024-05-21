classdef BodyClass

  %% PUBLIC PROPERTIES %%
  properties

    %%%%% Mass and Geometric Properties %%%%%

    S_b  % body maximum frontal area
    S_p  % body planform area
    S_s  % wetted area of the body
    f_ld % fineness ratio (length/max diameter)

    eta = 0.67 % cross-flow drag proportionality factor
    c_dc = 1.2 % cross-flow drag coefficient
    C_f  = 0.004 % turbulent flat-plate skin friction coefficient
    C_Db % base drag-coefficient
    C_Dfb % zero-lift drag

    mass % mass
    I    % moment of inertia
    cm_b % center of mass in the body frame
    wing_width % width of body at wing
    tail_width % width of body at tail

    %%%%%% Aerodynamic Forces and Moments %%%%%%

    Fx
    Fy
    Fz
    Mx
    My
    Mz
    
    Fx_x 
    Fy_x
    Fz_x
    Mx_x
    My_x
    Mz_x      

  end

  %% PUBLIC METHODS %%
  methods 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for the tiltwing class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = BodyClass(varargin)

    % set up the body object
    % varargin : 1 - mass
    %            2 - inertia matrix
    %            3 - center of mass in the body frame
    %            4 - body width at wing
    %            5 - body width at tail

      %% set the body properties
      if nargin == 7
        obj.mass  = varargin{1};
        obj.I     = varargin{2};
        obj.cm_b  = varargin{3};
        obj.S_b   = varargin{4};
        obj.S_p   = varargin{5};
        obj.S_s   = varargin{6};
        obj.f_ld  = varargin{7};
      else 
        error('incorrect number of input arguments')
      end

      %% set up the body aerodynamic parameters
      % zero-lift drag
      obj.C_Dfb = obj.C_f*( 1 + ...
                            60/(obj.f_ld)^3 + ...
                            0.0025*(obj.f_ld) )*obj.S_s/obj.S_b;
      
      % base drag-coefficient
      obj.C_Db = 0.029/sqrt(obj.C_Dfb);

      obj.Fx = 0;
      obj.Fy = 0;
      obj.Fz = 0;
      obj.Mx = 0;
      obj.My = 0;
      obj.Mz = 0;
     
      obj.Fx_x = zeros(1,6);
      obj.Fy_x = zeros(1,6);
      obj.Fz_x = zeros(1,6);
      obj.Mx_x = zeros(1,6);
      obj.My_x = zeros(1,6);
      obj.Mz_x = zeros(1,6);

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute the aerodynamic forces of the wing-prop combo
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    function obj = aero(obj, rho, uvw, om, cm_b, ders)

      % compute the magnitude of the velocity and its derivatives
      V = sqrt(sum(uvw.^2));
      if V == 0
        V_u = 0;
        V_v = 0;
        V_w = 0;
      else
        V_u = uvw(1)/V;
        V_v = uvw(2)/V;
        V_w = uvw(3)/V;
      end

      % Compute the dynamic pressure and its derivatives
      q = 0.5*rho*V^2;
      q_u = rho*V*V_u;
      q_v = rho*V*V_v;
      q_w = rho*V*V_w;

      % angle of attack
      alf = atan2(uvw(3),uvw(1)); 

      % take the derivatives with respect to the local velocity components
      if V == 0 
        alf_u = 0;
        alf_w = 0;
      else
        alf_u = 1./(1+(uvw(3)./uvw(1)).^2).*(-uvw(3)./uvw(1).^2);
        alf_w = 1./(1+(uvw(3)./uvw(1)).^2).*(1./uvw(1));
      end

      %% Z Force Coefficient  
      Cz = -sin(2*alf)*cos(alf/2)-obj.eta*obj.c_dc*obj.S_p/obj.S_b*sin(alf)^2;
      Cz_alf = -2*cos(2*alf)*cos(alf/2) + 0.5*sin(2*alf)*sin(alf/2)...
               - obj.eta*obj.c_dc*obj.S_p/obj.S_b*2*cos(alf)*sin(alf);

      %% X Force Coefficient  
      Cx= -(obj.C_f + obj.C_Db)*cos(alf)^2;
      Cx_alf = (obj.C_f+obj.C_Db)*2*sin(alf)*cos(alf);
      

      obj.Fx = q*Cx*obj.S_b;
      obj.Fy = 0;
      obj.Fz = q*Cz*obj.S_b;
      obj.Mx = 0;
      obj.My = 0;
      obj.Mz = 0;
    
      if ders
        Fx_q = Cx*obj.S_b;
        Fz_q = Cz*obj.S_b;

        Fx_Cx = q*obj.S_b;
        Fz_Cz = q*obj.S_b;

        Fx_u = Fx_q*q_u + Fx_Cx*Cx_alf*alf_u;
        Fx_w = Fx_q*q_w + Fx_Cx*Cx_alf*alf_w;

        Fz_u = Fz_q*q_u + Fz_Cz*Cz_alf*alf_u;
        Fz_w = Fz_q*q_w + Fz_Cz*Cz_alf*alf_w;

      
        obj.Fx_x = [ Fx_u 0 Fx_w 0 0 0];
        obj.Fy_x = [ 0 0 0 0 0 0];
        obj.Fz_x = [ Fz_u 0 Fz_w 0 0 0];

        obj.Mx_x = [ 0 0 0 0 0 0];
        obj.My_x = [ 0 0 0 0 0 0];
        obj.Mz_x = [ 0 0 0 0 0 0];
      end

    end

  end

end

