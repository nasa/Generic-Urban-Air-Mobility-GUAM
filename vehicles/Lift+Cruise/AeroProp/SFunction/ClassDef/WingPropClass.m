classdef WingPropClass

  properties

    %%%%%% Setting accessible to the User %%%%%%%

    tilt_angle = 0; % incedence angle
    om_prop         % propeller speed
    Thrust          % propeller thrust
    del_f = 0;      % flap defelection angle
    del_a = 0;      % aileron deflection angle
    h_b             % hinge location in the body frame
    g_hand;         % graphics handle(s)
    
  end

  properties (SetAccess = private)

    %%%%%% Wing and Propellers %%%%%%

    Wing       % Wing mass and geometry definition
    Props      % Propeller mass and geometry definitition

    %%%%%% Mass and Geometry Properties %%%%%%

    b_e        % exposed wing span
    semi_b_e   % exposed semi span of wing
    semi_c4_b  % quarter chord of left and right semi spans in the body frame 
    c4_h       % quarter chord in the hinge frame
    cm_b       % center of mass in the body frame
    mass       % total mass of the wing-prop combo
    I          % Inertia matrix of wing_prop combo about center of mass
    NP         % number of props

    %%%%%% Left and Right Wing-Prop combos %%%%%%

    Left % Left wing-prop combo
    Right % Right wing-prop combo

    %%%%%% Aerodynamic Forces and Moments %%%%%%

    Fx    %  
    Fy    % 
    Fz    % 
    Mx    % Roll Moment (Velocity Frame)
    My    % Pitch Moment (Velocity Frame)
    Mz    % Yaw Moment (Velocity Frame)

    Fx_x  %  
    Fy_x  % 
    Fz_x  %  
    Mx_x  % Roll Moment (Velocity Frame)
    My_x  % Pitch Moment (Velocity Frame)
    Mz_x  % Yaw Moment (Velocity Frame)

    Fx_u  %  
    Fy_u  % 
    Fz_u  %  
    Mx_u  % Roll Moment (Velocity Frame)
    My_u  % Pitch Moment (Velocity Frame)
    Mz_u  % Yaw Moment (Velocity Frame)

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

    function S = hat(obj,x)
      S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    end
    
  end


  methods

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for the WingProp Class 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = WingPropClass(varargin) 

    % set up the Wing-Propeller combination
    % varargin : 1 - wing
    %            2 - proppellers

      %% check to see if the proper number of 
      %% arguments were passed
      if nargin < 1 | nargin > 2
        error('incorrect number of input arguments')
      end
  
      %% Assign the Wing
      if isa(varargin{1}, 'WingClass')
        obj.Wing = varargin{1}; 
      else 
        error('first argument must be of the class ''WingClass''')
      end

      % set the exposed wing  and determine the location of the
      % quarter chord at the root of each wing
      obj.b_e = obj.Wing.b_e;
      y_diff = (obj.Wing.b - obj.b_e)/2;
      obj.semi_c4_b = zeros(3,2); % preallocate storage
      obj.semi_c4_b(:,1) = obj.Wing.c4_b  - [ 0; y_diff; 0];
      obj.semi_c4_b(:,2) = obj.Wing.c4_b  + [ 0; y_diff; 0];

      %% If no propellers are included build the left and right wing
      %% based on the wing parameters
      if nargin == 1

        %% set the number of props to 0
        obj.NP = 0;

        obj.Left = SemiWingPropClass('Left',...
                                obj.Wing.airfoil,...
                                obj.Wing.coeff,...
                                obj.semi_b_e,...
                                [obj.Wing.c_root obj.Wing.c_tip],...
                                obj.Wing.gamma,...
                                obj.Wing.y_flap,...
                                obj.Wing.y_aileron,...
                                obj.semi_c4_b(:,1));

        obj.Right = SemiWingPropClass('Right',...
                                obj.Wing.airfoil,...
                                obj.Wing.coeff,...
                                obj.semi_b_e,...
                                [obj.Wing.c_root obj.Wing.c_tip],...
                                obj.Wing.gamma,...
                                obj.Wing.y_flap,...
                                obj.Wing.y_aileron,...
                                obj.semi_c4_b(:,2));
      end
      
      %% If propeller are included build the left and
      %% right wing-propeller combinations
      if nargin == 2

        %% Assign the propeller object
        if isa(varargin{2}, 'PropellerClass')
          obj.Props = varargin{2}; 
        else 
          error('second argument must be of the class ''PropellerClass''')
        end

        %% Assume the wing is symmetric and the propellers are centered
        %% on the wing airfoil chord line
        obj.NP = numel(obj.Props.Dp);
        nPropsSemi = obj.NP/2;

        %% Propeller Coefficients
        prop_coef = obj.Props.coef(nPropsSemi+1:end);

        %% Propeller Coefficients
        prop_spin = obj.Props.spin(nPropsSemi+1:end);

        %% Propeller Diameters
        D = obj.Props.Dp(nPropsSemi+1:end);
        %% Propeller center locations on the wing span
        C = obj.Props.p_b(2,nPropsSemi+1:end) - repmat(obj.semi_c4_b(2,2),1,nPropsSemi);

        obj.Left = SemiWingPropClass('Left', ...
                                obj.Wing.airfoil, ...
                                obj.Wing.coeff, ...
                                obj.semi_b_e, ...
                                [obj.Wing.c_root obj.Wing.c_tip], ...
                                obj.Wing.gamma,...
                                obj.Wing.y_flap, ...
                                obj.Wing.y_aileron, ...
                                obj.semi_c4_b(:,1), ...
                                prop_coef, ...
                                -prop_spin,...
                                D, ...
                                C);

        obj.Right = SemiWingPropClass('Right', ...
                                obj.Wing.airfoil, ...
                                obj.Wing.coeff, ...
                                obj.semi_b_e, ...
                                [obj.Wing.c_root obj.Wing.c_tip], ...
                                obj.Wing.gamma,...
                                obj.Wing.y_flap, ...
                                obj.Wing.y_aileron, ...
                                obj.semi_c4_b(:,2), ...
                                prop_coef, ...
                                prop_spin,...
                                D, ...
                                C);

      end

      %% set the default hinge location at the wing quarter chord
      obj.h_b = obj.Wing.c4_b;
      
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
      
      obj.Fx_u = zeros(1,obj.NP+3); % NP + 3 surface inputs
      obj.Fy_u = zeros(1,obj.NP+3);
      obj.Fz_u = zeros(1,obj.NP+3);
      obj.Mx_u = zeros(1,obj.NP+3);
      obj.My_u = zeros(1,obj.NP+3);
      obj.Mz_u = zeros(1,obj.NP+3);

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the hing location in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.h_b(obj, value)
      obj.c4_h = obj.Wing.c4_b - value;
      obj.h_b = value;

      % update the hing position on the left and right side
      y_diff = (obj.Wing.b - obj.b_e)/2;
      obj.Left.c4_h = obj.c4_h - [0; y_diff; 0];
      obj.Right.c4_h = obj.c4_h + [0; y_diff; 0];

      obj.Left.h_b = obj.h_b;
      obj.Right.h_b = obj.h_b;

    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = h_c(obj, cm_b)
      r = obj.h_b - cm_b;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge coordinates in the velocity frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = h_v(obj, cm_b, alf)
      h_c = obj.h_c(cm_b);
      r = obj.Ry(alf)*h_c;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% semi span of exposed wing
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.semi_b_e(obj)
      r = obj.b_e/2;
    end
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Wing center of mass in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = w_cm_b(obj)

      if isempty(obj.h_b)
        r = obj.Wing.cm_b;
      else
        w_cm_h = obj.Wing.cm_b - obj.h_b;
        r = obj.h_b + obj.Ry(obj.tilt_angle)*w_cm_h;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Propeller center of mass in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = p_cm_b(obj)

      if isempty(obj.h_b)
        r = obj.Props.m_b;
      else
        p_cm_h = obj.Props.m_b - obj.h_b;
        r = obj.h_b + obj.Ry(obj.tilt_angle)*p_cm_h;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Mass of the wing and propeller motors
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.mass(obj)
      r = obj.Wing.mass;
%      if obj.NP > 0
      if numel(obj.Props) > 0
        r = sum([r obj.Props.mass]);
      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% center of mass in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.cm_b(obj)
      
      w_cm_b = obj.w_cm_b;
      w_m = obj.Wing.mass;

%      if obj.NP > 0
      if numel(obj.Props) > 0
        p_m = obj.Props.mass;
        p_cm_b = obj.p_cm_b;
      else  
        p_m = [];
        p_cm_b = [];
      end

      r = (1/obj.mass)*sum([w_cm_b*w_m  p_cm_b*diag(p_m)], 2);

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inertia matrix about the wing-prop center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.I(obj)

      r_w = obj.w_cm_b - obj.cm_b;
      Iw = obj.Ry(obj.tilt_angle)*obj.Wing.I*obj.Ry(obj.tilt_angle)';

      r = Iw - obj.Wing.mass*obj.hat(r_w)^2;

%      if obj.NP > 0
      if numel(obj.Props) > 0
        r_p = obj.p_cm_b - obj.cm_b;
        for ii = 1:obj.NP
          r = r - obj.Props.mass(ii)*obj.hat(r_p(:,ii))^2;
        end
      end

    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the propeller speed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.om_prop(obj,value)

      % chech the vector size and assign the thrust values
      if numel(value) ~= obj.NP 
        error('thrust vector size must equal the number of props')
      else
        obj.om_prop = value;
        obj.Left.om_prop = fliplr(value(1:obj.NP/2)); % flip (in->out)
        obj.Right.om_prop = value(obj.NP/2+1:end);
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Thrust(obj)
%      if obj.NP > 0 
      if numel(obj.Props) > 0 
        r = [fliplr(obj.Left.Thrust) obj.Right.Thrust];
      else 
        r = [];
      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the wing tilt angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.tilt_angle(obj, value)

      obj.tilt_angle = value;
      obj.Left.tilt_angle = value;
      obj.Right.tilt_angle = value; 

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the flap deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_f(obj, value)

      obj.del_f = value;
      obj.Left.del_f = value;
      obj.Right.del_f = value; 

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the aileron deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_a(obj, value)

      obj.del_a = value;
      obj.Left.del_a = value;
      obj.Right.del_a = value; 

    end
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute the aerodynamic forces of the wing-prop combo
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = aero(obj, rho, uvw, om, cm_b, ders)

      % get the aerodynamic contributions of the left and right
      % wing and add them up

      obj.Left = obj.Left.aero(rho, uvw, om, cm_b, ders);
      obj.Right = obj.Right.aero(rho, uvw, om, cm_b, ders);

      % obj.L = obj.Left.L + obj.Right.L;
      % obj.D = obj.Left.D + obj.Right.D;
      % obj.S = obj.Left.S + obj.Right.S;

      obj.Fx = obj.Left.Fx + obj.Right.Fx;
      obj.Fy = obj.Left.Fy + obj.Right.Fy;
      obj.Fz = obj.Left.Fz + obj.Right.Fz;

      obj.Mx = obj.Left.Mx + obj.Right.Mx;
      obj.My = obj.Left.My + obj.Right.My;
      obj.Mz = obj.Left.Mz + obj.Right.Mz;

      if ders
        obj.Fx_x  = obj.Left.Fx_x + obj.Right.Fx_x;
        obj.Fy_x  = obj.Left.Fy_x + obj.Right.Fy_x;
        obj.Fz_x  = obj.Left.Fz_x + obj.Right.Fz_x;
        obj.Mx_x  = obj.Left.Mx_x + obj.Right.Mx_x;
        obj.My_x  = obj.Left.My_x + obj.Right.My_x;
        obj.Mz_x  = obj.Left.Mz_x + obj.Right.Mz_x;

        obj.Fx_u(1,1:obj.NP) = [fliplr(obj.Left.Fx_u(1,1:obj.NP/2))  obj.Right.Fx_u(1,1:obj.NP/2)];
        obj.Fx_u(1,obj.NP+1:end) = obj.Left.Fx_u(1,obj.NP/2+1:end)+obj.Right.Fx_u(1,obj.NP/2+1:end);
        obj.Fy_u(1,1:obj.NP) = [fliplr(obj.Left.Fy_u(1,1:obj.NP/2))  obj.Right.Fy_u(1,1:obj.NP/2)];
        obj.Fy_u(1,obj.NP+1:end) = obj.Left.Fy_u(1,obj.NP/2+1:end)+obj.Right.Fy_u(1,obj.NP/2+1:end);
        obj.Fz_u(1,1:obj.NP) = [fliplr(obj.Left.Fz_u(1,1:obj.NP/2))  obj.Right.Fz_u(1,1:obj.NP/2)];
        obj.Fz_u(1,obj.NP+1:end) = obj.Left.Fz_u(1,obj.NP/2+1:end)+obj.Right.Fz_u(1,obj.NP/2+1:end);

        obj.Mx_u(1,1:obj.NP) = [fliplr(obj.Left.Mx_u(1,1:obj.NP/2))  obj.Right.Mx_u(1,1:obj.NP/2)];
        obj.Mx_u(1,obj.NP+1:end) = obj.Left.Mx_u(1,obj.NP/2+1:end)+obj.Right.Mx_u(1,obj.NP/2+1:end);
        obj.My_u(1,1:obj.NP) = [fliplr(obj.Left.My_u(1,1:obj.NP/2))  obj.Right.My_u(1,1:obj.NP/2)];
        obj.My_u(1,obj.NP+1:end) = obj.Left.My_u(1,obj.NP/2+1:end)+obj.Right.My_u(1,obj.NP/2+1:end);
        obj.Mz_u(1,1:obj.NP) = [fliplr(obj.Left.Mz_u(1,1:obj.NP/2))  obj.Right.Mz_u(1,1:obj.NP/2)];
        obj.Mz_u(1,obj.NP+1:end) = obj.Left.Mz_u(1,obj.NP/2+1:end)+obj.Right.Mz_u(1,obj.NP/2+1:end);
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw the wing-prop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [obj] = draw(obj, rho, V, cm_b)

      obj.Left = obj.Left.draw(rho, V, cm_b);
      obj.Right = obj.Right.draw(rho, V, cm_b);

    end

  end

end
      

