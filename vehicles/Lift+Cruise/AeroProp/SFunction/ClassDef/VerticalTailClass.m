classdef VerticalTailClass

  %% PUBLIC PROPERTIES %%
  properties

    %%%%%%% Semi Wing Class for the Vertical tail %%%%%%

    tail % semi wing prop for vertical tail
    c4_b % quarter chord location in the body frame

    %%%%%% Mass and geometric properties of the vertical tail %%%%%%

    mass  % mass of the vertical tail
    I     % Inertial matrix of the vertical tail
    cm_b  % center of mass in the body frame

    %%%%%% Aerodynamic Forces and moments %%%%%%%%

    Fx
    Fy
    Fz
    Mx % roll moment
    My % pitch moment
    Mz % yaw moment

    Fx_x  
    Fy_x  
    Fz_x  
    Mx_x 
    My_x 
    Mz_x 

    Fx_u  
    Fy_u  
    Fz_u  
    Mx_u 
    My_u 
    Mz_u 

    g_hand; % Graphics handle

  end

  %% PUBLIC PROPERTIES %%
  properties (SetAccess = private)

    %%%%% properties used for visualization %%%%%

    Li
    Di
    y
    ci

  end

  %% PRIVATE METHODS 
  methods (Access = private)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % define the rotation functions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function R = Rx(obj,x)
      R = [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
    end

    function R = Ry(obj,x)
      R = [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)];
    end

    function R = Rz(obj,x)
      R = [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
    end
    
  end

  %% PUBLIC METHODS
  methods

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for vertical tail class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = VerticalTailClass(varargin)
    % set up the wingProp 
    % varargin : 1 - airfoil
    %            2 - coeff
    %            3 - span 
    %            4 - c, or if uniformly tapered [c_root c_tip]
    %            5 - y_rudder
    %            6 - c4_b wing quarter chord
    %            7 - mass
    %            8 - Inertia matrix
    %            9 - center of mass in body frame
  
      %% Set the vertical tail parameters
      if nargin == 9

        % Assign vertical tail parameters
        airfoil = varargin{1};
        coeff = varargin{2};
        b = varargin{3};
        c = varargin{4};
        y_rudder = varargin{5};
        obj.c4_b   = varargin{6};

        % Buid the vertical tail
        obj.tail = SemiWingPropClass( 'Right', ...
                                      airfoil, ...
                                      coeff, ...
                                      b, ...
                                      c, ...
                                      pi/2,... % dihedral = 90 deg
                                      y_rudder, ...
                                      [0 0], ... % no aileron
                                      obj.c4_b);

        %% Assign the mass and geometry 
        obj.mass = varargin{7};
        obj.I = varargin{8};
        obj.cm_b   = varargin{9};

      else
        error('incorrect number of input arguments')
      end 

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

      obj.Fx_u = zeros(1,1); % only 'flap', e.g. elevator, surface input
      obj.Fy_u = zeros(1,1);
      obj.Fz_u = zeros(1,1);
      obj.Mx_u = zeros(1,1);
      obj.My_u = zeros(1,1);
      obj.Mz_u = zeros(1,1);

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% quarter chord relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = c4_c(obj, cm_b)
      r = obj.c4_b - cm_b;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% quarter chord coordinates in the velocity frame
    %% NOTE: sideslip angle not considered
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = c4_v(obj, cm_b, alf)
      c4_c = obj.c4_c(cm_b);
      r = obj.Ry(alf)*c4_c;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% aerodynamic forces and moment in the velocity frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = aero(obj, rho, uvw, om, cm_b, ders)

      %%%% THIS FUNCTION NEEDS SOME VERIFICATION %%%%

      % call the strip theory aerodynamics 
      
      obj.tail = obj.tail.aero(rho,uvw, om, cm_b, ders);

      obj.Fx = obj.tail.Fx;
      obj.Fy = obj.tail.Fy;
      obj.Fz = obj.tail.Fz; 
      obj.Mx = obj.tail.Mx;
      obj.My = obj.tail.My;
      obj.Mz = obj.tail.Mz;

      if ders
        obj.Fx_x = obj.tail.Fx_x;
        obj.Fy_x = obj.tail.Fy_x;
        obj.Fz_x = obj.tail.Fz_x;
        obj.Mx_x = obj.tail.Mx_x;
        obj.My_x = obj.tail.My_x;
        obj.Mz_x = obj.tail.Mz_x;

        obj.Fx_u = obj.tail.Fx_u(:,1);
        obj.Fy_u = obj.tail.Fy_u(:,1);
        obj.Fz_u = obj.tail.Fz_u(:,1);
        obj.Mx_u = obj.tail.Mx_u(:,1);
        obj.My_u = obj.tail.My_u(:,1);
        obj.Mz_u = obj.tail.Mz_u(:,1);
      end


      % % Mostly used in visualizations
      % obj.Li = obj.tail.Li;
      % obj.Di = obj.tail.Di;

    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw the vertical tail
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = draw(obj, rho, V, cm_b)

      alf = atan2(V(3),V(1));
        
      % hinge and quarter chord in velocity frame
      c4_v = obj.c4_v(cm_b, alf);

      xw_ = -obj.tail.airfoil(:,1) + .25;
      zw_ =  obj.tail.airfoil(:,2);


      x_vt_ = repmat(xw_,1,numel(obj.tail.yc))*diag(obj.tail.cc);
      shift = x_vt_(end,1) - x_vt_(end,:);
      x_vt_ = x_vt_ + repmat(shift, numel(xw_), 1);
      y_vt = repmat(zw_,1,numel(obj.tail.yc))*diag(obj.tail.cc);
      z_vt_ = repmat(-obj.tail.yc',numel(xw_),1);

      x_vt =  x_vt_*cos(alf) + z_vt_*sin(alf);
      z_vt = -x_vt_*sin(alf) + z_vt_*cos(alf);


      Xtv = x_vt + c4_v(1);
      Ztv = z_vt + c4_v(3);
      Ytv = y_vt + c4_v(2);

      Ltv = repmat(obj.tail.Li(:,1)', numel(xw_),1);

      t_hand = surf(Xtv,Ytv,Ztv,Ltv,'edgecolor','none');
      obj.g_hand = t_hand;
    end
  
  end

end

