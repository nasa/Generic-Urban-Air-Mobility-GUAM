classdef TailClass
  properties

    %%%%%% Setting accessible to the User %%%%%%%

    tilt_angle = 0  % incedence angle
    om_prop     % propeller speed
    Thrust      % thrust
    del_e = 0;  % elevator deflection angle
    del_r = 0;  % rudder deflection angle
    h_b         % hinge location in the body frame

  end

  properties (SetAccess = private)

    %%%%% Tail Components %%%%%

    Horz      % Horizontal Tail mass and geometry  
    Vert      % Vertical Tail mass and geometry
    Props     % Propeller mass and geometry 

    %%%%% Horizontal Tail Properties %%%%%

    b_e       % exposed wing span
    semi_b_e  % exposed semi span
    semi_c4_b % location of the root c/4 of semi wing
    c4_h      % quarter chord in the hinge frame
    Left      % left semi wing-prop of horizontal tail
    Right     % right semi wing-prop of horizontal tail
    NP    % number of props

    %%%%% Tail Mas and Geometry Properties %%%%%

    cm_b % center of mass in the body frame
    mass % total mass in the body frame
    I    % inertia matrix of the tail

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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for a Tail Class Object
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = TailClass(varargin)

    % set up the tail
    % varargin : 1 - horizontal tail
    %            2 - propellers
    %            2 - vertical tail

      %% check to see if the proper number of 
      %% arguments were passed
      if nargin < 2 || nargin > 3
        error('incorrect number of input arguments')
      end

      %% Assign the Wing
      if isa(varargin{1}, 'WingClass')
        obj.Horz = varargin{1}; 
      else 
        error('first argument must be of the class ''WingClass''')
      end

      % initialize the tilt angle at zero
%      obj.tilt_angle = 0;
      
      % get the exposed wing and the semi span distances
      obj.b_e = obj.Horz.b_e;
      y_diff = (obj.Horz.b - obj.b_e)/2;
      obj.semi_c4_b = zeros(3,2); % preallocate storage
      obj.semi_c4_b(:,1) = obj.Horz.c4_b  - [ 0; y_diff; 0];
      obj.semi_c4_b(:,2) = obj.Horz.c4_b  + [ 0; y_diff; 0];

      %% If there are no propellers included build horizontal 
      %% and vertical tail with out propellers
      if nargin == 2
  
        %% set the number of props to 0
        obj.NP = 0;

        obj.Left = SemiWingPropClass('Left',...
                                obj.Horz.airfoil,...
                                obj.Horz.coeff,...
                                obj.semi_b_e,...
                                [obj.Horz.c_root obj.Horz.c_tip],...
                                obj.Horz.gamma,...
                                obj.Horz.y_flap,...
                                obj.Horz.y_aileron,...
                                obj.semi_c4_b(:,1));

        obj.Right = SemiWingPropClass('Right',...
                                obj.Horz.airfoil,...
                                obj.Horz.coeff,...
                                obj.semi_b_e,...
                                [obj.Horz.c_root obj.Horz.c_tip],...
                                obj.Horz.gamma,...
                                obj.Horz.y_flap,...
                                obj.Horz.y_aileron,...
                                obj.semi_c4_b(:,2));

        % assign the vertical tail
        if isa(varargin{2}, 'VerticalTailClass')  
          obj.Vert = varargin{2};
        else
          error('second argument must be of class ''VerticalTailClass''');
        end

      end


      %% If there are propellers included build horizontal 
      %% and vertical tail with propellers
      if nargin == 3

        if isa(varargin{2}, 'PropellerClass')
          obj.Props = varargin{2};
        else
          error('second argument must be of class ''PropellerClass''');
        end

        % determine the number and props and split them into 
        % the left and right sides
        obj.NP = numel(obj.Props.Dp);
        nPropsSemi = obj.NP/2;

        %% Propeller Coefficients
        prop_coef = obj.Props.coef(nPropsSemi+1:end);
        %% Propeller Diameters

        D = obj.Props.Dp(nPropsSemi+1:end);
        %% Propeller center locations on the wing span
        C = obj.Props.p_b(2,nPropsSemi+1:end) - repmat(obj.semi_c4_b(2,2),1,nPropsSemi);

        obj.Left = SemiWingPropClass('Left',...
                                obj.Horz.airfoil,...
                                obj.Horz.coeff,...
                                obj.semi_b_e,...
                                [obj.Horz.c_root obj.Horz.c_tip],...
                                obj.Horz.gamma,...
                                obj.Horz.y_flap,...
                                obj.Horz.y_aileron,...
                                obj.semi_c4_b(:,1),...
                                prop_coef,...
                                D,...
                                C);
       
        obj.Right = SemiWingPropClass('Right',...
                                obj.Horz.airfoil,...
                                obj.Horz.coeff,...
                                obj.semi_b_e,...
                                [obj.Horz.c_root obj.Horz.c_tip],...
                                obj.Horz.gamma,...
                                obj.Horz.y_flap,...
                                obj.Horz.y_aileron,...
                                obj.semi_c4_b(:,2),...
                                prop_coef,...
                                D,...
                                C);

        % assign the vertical tail
        if isa(varargin{3}, 'VerticalTailClass')
          obj.Vert = varargin{3};
        else
          error('third argument must be of class ''VerticalTailClass''');
        end

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

      obj.c4_h = obj.Horz.c4_b - value;
      obj.h_b = value;

      % update the hing position on the left and right side
      y_diff = (obj.Horz.b - obj.b_e)/2;
      if (isa(obj.Left,'SemiWingPropClass'))
        obj.Left.c4_h = obj.c4_h - [0; y_diff; 0];
        obj.Left.h_b = obj.h_b;
      end
      if (isa(obj.Right,'SemiWingPropClass'))
        obj.Right.c4_h = obj.c4_h + [0; y_diff; 0];
        obj.Right.h_b = obj.h_b;
      end

    end
      

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% semi span of exposed wing
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.semi_b_e(obj)

      r = obj.b_e/2;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% center of mass of the horizontal tail in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = ht_cm_b(obj)

      if isempty(obj.h_b)
        r = obj.Horz.cm_b;
      else
        ht_cm_h = obj.Horz.cm_b - obj.h_b;
        r = obj.h_b + obj.Ry(obj.tilt_angle)*ht_cm_h;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Propeller center of mass in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = pt_cm_b(obj)

      if isempty(obj.h_b)
        r = obj.Props.m_b;
      else
        pt_cm_h = obj.Props.m_b - obj.h_b;
        r = obj.h_b + obj.Ry(obj.tilt_angle)*pt_cm_h;
      end

    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% vertical tail center of mass in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = vt_cm_b(obj)

      r = obj.Vert.cm_b;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% mass of the tail assembly
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.mass(obj)

      r = obj.Horz.mass + obj.Vert.mass;
%      if obj.NP > 0
      if numel(obj.Props) > 0
        r = sum([r  obj.Props.mass]);
      end  

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % center of mass of the tail assembly in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.cm_b(obj)

      %% horizontal tail
      ht_cm_b = obj.ht_cm_b;
      ht_m = obj.Horz.mass;

      %% vertical tail
      vt_cm_b = obj.vt_cm_b;
      vt_m = obj.Vert.mass;

      %% propellers
%      if obj.NP > 0
      if numel(obj.Props) > 0
        pt_m = obj.Props.mass;
        pt_cm_b = obj.pt_cm_b;
      else  
        pt_m = [];
        pt_cm_b = [];
      end

      %% compute the center of mass in the body frame
      r = (1/obj.mass)*sum([ht_cm_b*ht_m pt_cm_b*diag(pt_m) vt_cm_b*vt_m],2);

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Moment of Inertia about the cm
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.I(obj)

      %% horizontal tail 
      r_ht = obj.ht_cm_b - obj.cm_b;
      Iht = obj.Ry(obj.tilt_angle)*obj.Horz.I*obj.Ry(obj.tilt_angle)';

      %% vertical tail
      r_vt = obj.vt_cm_b - obj.cm_b;
      Ivt = obj.Vert.I;
  
      %% combined inertia
      r = Iht - obj.Horz.mass*obj.hat(r_ht)^2 + ...
          Ivt - obj.Vert.mass*obj.hat(r_vt)^2;

      %% add in inertial due to props
%      if obj.NP > 0
      if numel(obj.Props) > 0
        pt_cm_b = obj.pt_cm_b;
        for ii = 1:obj.NP
          r_pt = pt_cm_b(:,ii) - obj.cm_b;
          r = r - obj.Props.motor_mass(ii)*obj.hat(r_pt)^2;
        end
      end

    end
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the propeller speed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.om_prop(obj, value)

      if numel(value) ~= obj.NP 
        error('thrust vector size must equal the number of props')
      else
        obj.om_prop = value;
        if (isa(obj.Left,'SemiWingPropClass'))
          obj.Left.om_prop = fliplr(value(1:obj.NP/2)); % flip (in->out)
        end
        if (isa(obj.Right,'SemiWingPropClass'))
          obj.Right.om_prop = value(obj.NP/2+1:end);
        end
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Thrust(obj)

%      if obj.NP > 0 
      if numel(obj.Props) > 0
       r = [ fliplr(obj.Left.Thrust)  obj.Right.Thrust ];
      else
        r = [];
      end

    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the tail incidence angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.tilt_angle(obj, value)

      obj.tilt_angle = value;
      if (isa(obj.Left,'SemiWingPropClass'))
        obj.Left.tilt_angle = value;
      end
      if (isa(obj.Right,'SemiWingPropClass'))
        obj.Right.tilt_angle = value; 
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the elevator deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_e(obj, value)

      obj.del_e = value;
      if (isa(obj.Left,'SemiWingPropClass'))
        obj.Left.del_f = value;
      end
      if (isa(obj.Right,'SemiWingPropClass'))
        obj.Right.del_f = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the rudder deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_r(obj, value)

      obj.del_r = value;
      if (isa(obj.Vert,'VerticalTailClass') && isa(obj.Vert.tail,'SemiWingPropClass'))
        obj.Vert.tail.del_f = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Tail aerodynamic forces and moments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = aero(obj, rho, uvw, om, cm_b, ders)

      % velocity
      if numel(uvw) == 3
        v_b = uvw;
        v_l = uvw;
        v_r = uvw;
      elseif numel(uvw) > 3
        v_b = uvw(:,1);
        v_l = uvw(:,2);
        v_r = uvw(:,3);
      else
        error('the input v is the wrong size');
      end

      %% get aerodynamics of each component
      obj.Left = obj.Left.aero(rho, v_l, om, cm_b, ders);
      obj.Right = obj.Right.aero(rho, v_r, om, cm_b, ders);
      obj.Vert = obj.Vert.aero(rho, v_b, om, cm_b, ders);

      %% sum the forces and moments
      obj.Fx = obj.Left.Fx + obj.Right.Fx + obj.Vert.Fx;
      obj.Fy = obj.Left.Fy + obj.Right.Fy + obj.Vert.Fy;
      obj.Fz = obj.Left.Fz + obj.Right.Fz + obj.Vert.Fz;
      obj.Mx = obj.Left.Mx + obj.Right.Mx + obj.Vert.Mx;
      obj.My = obj.Left.My + obj.Right.My + obj.Vert.My;
      obj.Mz = obj.Left.Mz + obj.Right.Mz + obj.Vert.Mz;
  
      if ders
        obj.Fx_x = obj.Left.Fx_x + obj.Right.Fx_x + obj.Vert.Fx_x;
        obj.Fy_x = obj.Left.Fy_x + obj.Right.Fy_x + obj.Vert.Fy_x;
        obj.Fz_x = obj.Left.Fz_x + obj.Right.Fz_x + obj.Vert.Fz_x;
        obj.Mx_x = obj.Left.Mx_x + obj.Right.Mx_x + obj.Vert.Mx_x;
        obj.My_x = obj.Left.My_x + obj.Right.My_x + obj.Vert.My_x;
        obj.Mz_x = obj.Left.Mz_x + obj.Right.Mz_x + obj.Vert.Mz_x;

        % left side Props, right side props
        obj.Fx_u(1,1:obj.NP) = [fliplr(obj.Left.Fx_u(1:obj.NP/2)) obj.Right.Fx_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.Fx_u(1,obj.NP+1:end) = [obj.Left.Fx_u(:,obj.NP/2+1)+obj.Right.Fx_u(obj.NP/2+1) obj.Vert.Fx_u(1) obj.Left.Fx_u(end)+obj.Right.Fx_u(end)];

        % left side Props, right side props
        obj.Fy_u(1,1:obj.NP) = [fliplr(obj.Left.Fy_u(1:obj.NP/2)) obj.Right.Fy_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.Fy_u(1,obj.NP+1:end) = [obj.Left.Fy_u(:,obj.NP/2+1)+obj.Right.Fy_u(obj.NP/2+1) obj.Vert.Fy_u(1) obj.Left.Fy_u(end)+obj.Right.Fy_u(end)];

        % left side Props, right side props
        obj.Fz_u(1,1:obj.NP) = [fliplr(obj.Left.Fz_u(1:obj.NP/2)) obj.Right.Fz_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.Fz_u(1,obj.NP+1:end) = [obj.Left.Fz_u(:,obj.NP/2+1)+obj.Right.Fz_u(obj.NP/2+1) obj.Vert.Fz_u(1) obj.Left.Fz_u(end)+obj.Right.Fz_u(end)];

        % left side Props, right side props
        obj.Mx_u(1,1:obj.NP) = [fliplr(obj.Left.Mx_u(1:obj.NP/2)) obj.Right.Mx_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.Mx_u(1,obj.NP+1:end) = [obj.Left.Mx_u(:,obj.NP/2+1)+obj.Right.Mx_u(obj.NP/2+1) obj.Vert.Mx_u(1) obj.Left.Mx_u(end)+obj.Right.Mx_u(end)];

        % left side Props, right side props
        obj.My_u(1,1:obj.NP) = [fliplr(obj.Left.My_u(1:obj.NP/2)) obj.Right.My_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.My_u(1,obj.NP+1:end) = [obj.Left.My_u(:,obj.NP/2+1)+obj.Right.My_u(obj.NP/2+1) obj.Vert.My_u(1) obj.Left.My_u(end)+obj.Right.My_u(end)];

        % left side Props, right side props
        obj.Mz_u(1,1:obj.NP) = [fliplr(obj.Left.Mz_u(1:obj.NP/2)) obj.Right.Mz_u(1:obj.NP/2)];
        % elevator, rudder, tilt
        obj.Mz_u(1,obj.NP+1:end) = [obj.Left.Mz_u(:,obj.NP/2+1)+obj.Right.Mz_u(obj.NP/2+1) obj.Vert.Mz_u(1) obj.Left.Mz_u(end)+obj.Right.Mz_u(end)];
      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw the Tail
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = draw(obj, rho, V, cm_b)

       obj.Left = obj.Left.draw(rho, V, cm_b);
       obj.Right = obj.Right.draw(rho, V, cm_b);
       obj.Vert = obj.Vert.draw(rho, V, cm_b);
    end

  end

end
    
