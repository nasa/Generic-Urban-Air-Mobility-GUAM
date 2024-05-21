classdef TiltWingClass %#codegen
  
  %% PUBLIC PROPERTIES %%
  properties (SetAccess = public)

    %%%%% Tiltwing Aircraft Components %%%%%

    WingProp % wing-propeller combination
    Tail     % tail (horizontal and vertical)
    Body     % body 

    Prop     % extra propellers
    Masses   % extra masses

    %%%%% Propeller Thrust %%%%%

    om_w % wing propellers
    om_t % tail propellers
    om_p % extra propellers

    Tw  % wing propellers
    Tt  % tail propellers
    Tp  % extra propellers
    
    Qp % torque of extra propellers

    %%%%% Control Surface Deflections %%%%%

    del_a = 0; % aileron deflection angle
    del_f = 0; % flap deflection angle
    del_e = 0; % elevator deflection angle
    del_r = 0; % rudder deflection angle

    %%%%% Wing and Tail Tilt Angles %%%%%

    i_w = 0; % wing tilt angle 
    i_t = 0; % tail tilt angle

    %%%%% Tail Downwash Limits %%%%%%
    dw_l = -3*pi/180; % lower limit
    dw_u = 17*pi/180; % upper limit

    %%%% Graphics Handles %%%%%%%%%%
    g_hand; % graphics handles for draw object
  

  end

  %% PRIVATE PROPERTIES %%
  properties (SetAccess = private)

    %%%%% Mass and Geometric Properties %%%%%

    mass % mass of the aircraft
    I    % moment of inertia of the aircraft
    cm_b % aircraft center of mass in the body frame

    %%%%% Total Aerodynamic/Propulsion Forces and Moments %%%%%

    L
    D

    Fx
    Fy
    Fz
    Mx % roll moment
    My % pitching moment
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

    %%%%% Aerodynamic Forces and Moments %%%%%
    
    aero_Fx
    aero_Fy
    aero_Fz
    aero_Mx
    aero_My
    aero_Mz

    %%%%% Propulsion Forces and Moments %%%%%
    
    prop_Fx
    prop_Fy
    prop_Fz
    prop_Mx
    prop_My
    prop_Mz
    
  end

  %% PRIVATE METHODS %%
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

  %% PUBLIC METHODS %%
  methods

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for the tiltwing class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = TiltWingClass(varargin) 

    % set up the tiltwing aircraft 
    % varargin : 1 - Forward wing propeller combination
    %            2 - Tail
    %            3 - Body
    %            4 - Extra Propellers
    %            5 - Extra MAsses

      %% check to see if the proper number of 
      %% arguments were passed
      if nargin < 2 || nargin > 5
        error('incorrect number of input arguments')
      end

      %% Assign the Wing
      if isa(varargin{1}, 'WingPropClass')
        obj.WingProp = varargin{1}; 
      else 
        error('first argument must be of the class ''WingPropClass''')
      end

      %% Assign the Tail
      if isa(varargin{2}, 'TailClass')
        obj.Tail = varargin{2}; 
      else 
        error('second argument must be of the class ''TailClass''')
      end

      %% Assign the body
      if nargin >= 3

        if isa(varargin{3},'BodyClass') || isempty(varargin{3})
          obj.Body = varargin{3}; 
        else
          error('third argument must be of the class ''BodyClass'' or empty []...')
        end

      end

      %% Assign the extra propellers
      if nargin >= 4

        if (isa(varargin{4},'cell') && isa(varargin{4}{1},'PropellerClass')) || isempty(varargin{4})
          obj.Prop = varargin{4};
          obj.om_p = zeros(1,numel(varargin{4}));
        else
          error('fourth argument must be a cell array of the class ''PropellerClass'' or empty []...')
        end

      end

      %% Assign the extra masses
      if nargin == 5
  
        if (isa(varargin{5},'cell') && isa(varargin{5}{1}, 'MassClass')) || isempty(varargin{5})
          obj.Masses = varargin{5};
        else
          error('fifth argument must be a cell array of the class ''MassClass'' or empty []...')
        end
  
      end
      
      obj.L = 0;
      obj.D = 0;

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
      
      obj.Fx_u = zeros(1,42);
      obj.Fy_u = zeros(1,42);
      obj.Fz_u = zeros(1,42);
      obj.Mx_u = zeros(1,42);
      obj.My_u = zeros(1,42);
      obj.Mz_u = zeros(1,42);

      obj.aero_Fx = 0;
      obj.aero_Fy = 0;
      obj.aero_Fz = 0;
      obj.aero_Mx = 0;
      obj.aero_My = 0;
      obj.aero_Mz = 0;

      obj.prop_Fx = 0;
      obj.prop_Fy = 0;
      obj.prop_Fz = 0;
      obj.prop_Mx = 0;
      obj.prop_My = 0;
      obj.prop_Mz = 0;
      
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the wing tilt angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.i_w(obj, value)

      % limit the wing incidence angle from 0-90 deg
      if value <= pi/2 && value >= 0
        obj.i_w = value;
        obj.WingProp.tilt_angle = value;
      else
        error('Wing incidence angle must be between 0 and 90 deg');
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the tail tilt angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.i_t(obj, value)

      % limit the tail incidence angle from 0-90 deg
      if value <= pi/2 && value >= 0
        obj.i_t = value;
        obj.Tail.tilt_angle = value;
      else
        error('Tail incidence angle must be between 0 and 90 deg');
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the aileron deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_a(obj, value)

      if abs(value) < 30*pi/180
        obj.del_a = value;
        obj.WingProp.del_a = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the flap deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_f(obj, value)

      if abs(value) < 30*pi/180 
        obj.del_f = value;
        obj.WingProp.del_f = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the elevator deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_e(obj, value)

      if abs(value) < 30*pi/180
        obj.del_e = value;
        obj.Tail.del_e = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the rudder deflection angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.del_r(obj, value)

      if abs(value) < 30*pi/180
        obj.del_r = value;
        obj.Tail.del_r = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the wing propeller speed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.om_w(obj, value)

      % assuming symmetry there should be an even number
      if mod(numel(value),2)
        error('the wing thrust vector must be even')
      else
        obj.om_w = value;
        obj.WingProp.om_prop = value;
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the tail propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.om_t(obj, value)

      % assume that the propellers are symetric and 
      % therefore there should be an even number
      if mod(numel(value),2)
        error('the wing thrust vector must be even')
      else
        obj.om_t = value;
        obj.Tail.om_prop = value;

      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set the thrust at the extra propellers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.om_p(obj, value)
    
      if ~isempty(obj.Prop) 
        obj.om_p = value;
        
        for ii = 1:numel(obj.Prop)
          obj.Prop{ii}.om_prop = value(ii);
        end

      else
        error('No propellers are defined');
      end

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% total forces/moments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = total_Fb(obj)
      %% Forces in the wind frame
      r = [  obj.Fx;  obj.Fy;  obj.Fz ];
    end

    function r = total_Mb(obj)
      %% Moments in the wind frame
      r = [  obj.Mx;  obj.My;  obj.Mz ];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% aerodynamic forces/moments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = aero_Fb(obj)
      r = [obj.aero_Fx; obj.aero_Fy; obj.aero_Fz;];
    end

    function r = aero_Mb(obj)
      r = [obj.aero_Mx; obj.aero_My; obj.aero_Mz;];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propulsion forces/moments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = prop_Fb(obj)
      r = [obj.prop_Fx; obj.prop_Fy; obj.prop_Fz;];
    end

    function r = prop_Mb(obj)
      r = [obj.prop_Mx; obj.prop_My; obj.prop_Mz;];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the wing propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Tw(obj)
       r = obj.WingProp.Thrust;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the tail propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Tt(obj)
      r = obj.Tail.Thrust;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the thrust at the extra propellers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Tp(obj)
      if isempty(obj.om_p)
        r = zeros(1,1);
      else
        numProps = numel(obj.Prop);
        r = zeros(1,numProps);
        for ii = 1:numProps
          r(ii) = obj.Prop{ii}.T;
        end
      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the torque at the extra propellers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.Qp(obj)
      if isempty(obj.om_p)
        r = zeros(1,1);
      else
        numProps = numel(obj.Prop);
        r = zeros(1,numProps);
        for ii = 1:numProps
          r(ii) = obj.Prop{ii}.Q;
        end
      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get the mass of the entire aircraft
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.mass(obj)
      
      %% always include the wing and tail
      r = sum([obj.WingProp.mass obj.Tail.mass]); 
      
      %% include extra props if present
      if ~isempty(obj.Prop)
        for ii = 1:numel(obj.Prop)
          r = r + obj.Prop{ii}.mass;
        end
      end 
      
      %% include the body if present
      if ~isempty(obj.Body)
        r = r + obj.Body.mass;
      end
  
      %% include extra masses if present
      if ~isempty(obj.Masses)
        for ii = 1:numel(obj.Masses)
          r = r + obj.Masses{ii}.mass;
        end
      end
      
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get the CG of the aircraft in the body frame 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.cm_b(obj)

      %% always include the wing and tail
      w_cm_b = obj.WingProp.cm_b;
      t_cm_b = obj.Tail.cm_b;
      w_m = obj.WingProp.mass;
      t_m = obj.Tail.mass;
  
      %% include extra props if present
      numProps = numel(obj.Prop);
      p_cm_b = zeros(3,numProps);
      p_mass = zeros(1,numProps);
      if isempty(obj.Prop)
        p_cm_b = [0; 0; 0];
        p_m = 0;
      else
        for ii = 1:numProps
          p_cm_b(:,ii) = obj.Prop{ii}.cm_b;
          p_mass(ii) = obj.Prop{ii}.mass;
        end
        p_m = diag(p_mass);
      end

      %% include the body if present
      if isempty(obj.Body)
        b_cm_b = [0; 0; 0];
        b_m = 0;
      else
        b_cm_b = obj.Body.cm_b;
        b_m = obj.Body.mass;
      end

      %% include extra masses if present
      numMasses = numel(obj.Masses);
      m_cm_b = zeros(3,numMasses);
      m_mass = zeros(1,numMasses);
      if isempty(obj.Masses)
        m_cm_b = [0; 0; 0];
        m_m = 0;
      else
        for ii = 1:numMasses
          m_cm_b(:,ii) = obj.Masses{ii}.cm_b;
          m_mass(ii) = obj.Masses{ii}.mass;
        end
        m_m = diag(m_mass);
      end

      %% compute center of gravity in the body frame
      r = (1/obj.mass)*sum([w_cm_b*w_m...
                            t_cm_b*t_m...
                            p_cm_b*p_m...
                            b_cm_b*b_m...
                            m_cm_b*m_m],2);

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get the inertia matrix of the aircraft
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.I(obj)
      
      %% Always include the wing and tail
      % get wing and tail center of mass relative the
      % the aircraft center of mass
      r_w = obj.WingProp.cm_b - obj.cm_b;
      r_t = obj.Tail.cm_b - obj.cm_b;

      % compute the inertia matrix from the wing and tail
      r = obj.WingProp.I - obj.WingProp.mass*obj.hat(r_w)^2 + ...
          obj.Tail.I     - obj.Tail.mass*obj.hat(r_t)^2;

      % include the body if present
      if isempty(obj.Body)
        r_b = [0 0 0];
      else
        r_b = obj.Body.cm_b - obj.cm_b;
        r = r + obj.Body.I - obj.Body.mass*obj.hat(r_b)^2;
      end

      % include extra props if present
      nP = numel(obj.Prop);
      for ii = 1:nP
        r_p = obj.Prop{ii}.cm_b - obj.cm_b;
        r = r + obj.Prop{ii}.I - obj.Prop{ii}.mass*obj.hat(r_p)^2;
      end

      % include extra masses if present
      nM = numel(obj.Masses);
      for ii = 1:nM
        r_m = obj.Masses{ii}.cm_b - obj.cm_b;
        r = r + obj.Masses{ii}.I - obj.Masses{ii}.mass*obj.hat(r_m)^2;
      end
    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Tiltwing aerodynamic forces and moment             
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = aero(obj, rho, uvw, om, ders)

      % %% get the angle of attack at the wing
      % alf_w = alf+obj.i_w;
      %                         
      % %% Take into account the downwash of the 
      % %% forward wing on the tail
      % 
      % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % % At the moment we just consider the most inside
      % % propeller and assume that the downwash at the 
      % % tail is due solely to it.
      % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % 
      % %% Left side
      % left  = obj.WingProp.Left;
      % [ vs_l alfs_l ] = left.Slipstream( rho, ...
      %                                    v, ...
      %                                    alf_w, ...
      %                                    left.Thrust(1), ...
      %                                    left.A(1));
      % dw_l = obj.i_w - alfs_l;
      % if ( dw_l > obj.dw_l & dw_l < obj.dw_u )
      %   alf_l = -obj.i_w + alfs_l;
      %   v_l = vs_l;
      % else
      %   alf_l = alf;
      %   v_l = v;
      % end 

      % %% Right side
      % right = obj.WingProp.Right;
      % [ vs_r alfs_r ] = right.Slipstream( rho, ...
      %                                     v, ...
      %                                     alf_w, ...
      %                                     right.Thrust(1), ...
      %                                     right.A(1));
      % dw_r = obj.i_w - alfs_r; 
      % if ( dw_r > obj.dw_l & dw_r < obj.dw_u )
      %   alf_r = -obj.i_w + alfs_r;
      %   v_r = vs_r;
      % else
      %   alf_r = alf;
      %   v_r = v;
      % end 

      %% Compute the aerodynamics of the wing and tail
      %obj.Tail = obj.Tail.aero(rho,[v v_l v_r], [alf alf_l alf_r], beta, om, obj.cm_b);
      obj.WingProp = obj.WingProp.aero(rho, uvw, om, obj.cm_b, ders);
      obj.Tail = obj.Tail.aero(rho,uvw, om, obj.cm_b, ders); % assume no wing wake affecting tail

      %% Sum the wing and tail aerodynamic forces and moments
      obj.Fx = obj.WingProp.Fx + obj.Tail.Fx;
      obj.Fy = obj.WingProp.Fy + obj.Tail.Fy;
      obj.Fz = obj.WingProp.Fz + obj.Tail.Fz;
      obj.Mx = obj.WingProp.Mx + obj.Tail.Mx;
      obj.My = obj.WingProp.My + obj.Tail.My;
      obj.Mz = obj.WingProp.Mz + obj.Tail.Mz;

      if ders

        % state partial derivatives
        obj.Fx_x = obj.WingProp.Fx_x + obj.Tail.Fx_x;
        obj.Fy_x = obj.WingProp.Fy_x + obj.Tail.Fy_x;
        obj.Fz_x = obj.WingProp.Fz_x + obj.Tail.Fz_x;
        obj.Mx_x = obj.WingProp.Mx_x + obj.Tail.Mx_x;
        obj.My_x = obj.WingProp.My_x + obj.Tail.My_x;
        obj.Mz_x = obj.WingProp.Mz_x + obj.Tail.Mz_x;

        % input partial derivatives
        obj.Fx_u(1,1:6) = [ obj.WingProp.Fx_u  obj.Tail.Fx_u ]; % WingProp size is 1x(NP+3), Tail size is 1x(NP+3), so each 1x3 in this case
        obj.Fy_u(1,1:6) = [ obj.WingProp.Fy_u  obj.Tail.Fy_u ];
        obj.Fz_u(1,1:6) = [ obj.WingProp.Fz_u  obj.Tail.Fz_u ];
        obj.Mx_u(1,1:6) = [ obj.WingProp.Mx_u  obj.Tail.Mx_u];
        obj.My_u(1,1:6) = [ obj.WingProp.My_u  obj.Tail.My_u];
        obj.Mz_u(1,1:6) = [ obj.WingProp.Mz_u  obj.Tail.Mz_u];
      end


      %% Add in the Body effects
      % for now we just used some projected area to estimate the
      % increase in drag
      if ~isempty(obj.Body)
        obj.Body = obj.Body.aero(rho, uvw, om, obj.cm_b, ders);

        obj.Fx = obj.Fx + obj.Body.Fx;
        obj.Fy = obj.Fy + obj.Body.Fy;
        obj.Fz = obj.Fz + obj.Body.Fz;

        obj.Mx = obj.Mx + obj.Body.Mx;
        obj.My = obj.My + obj.Body.My;
        obj.Mz = obj.Mz + obj.Body.Mz;

        if ders
          obj.Fx_x = obj.Fx_x + obj.Body.Fx_x;
          obj.Fy_x = obj.Fy_x + obj.Body.Fy_x;
          obj.Fz_x = obj.Fz_x + obj.Body.Fz_x;
          obj.Mx_x = obj.Mx_x + obj.Body.Mx_x;
          obj.My_x = obj.My_x + obj.Body.My_x;
          obj.Mz_x = obj.Mz_x + obj.Body.Mz_x;
        end

      end

      %% total aerodynamic forces/moments
      % underlying SemiWingPropClass needs modification to support separate
      % aerodynamic and propulsion forces/moments for tiltwing operation
      obj.aero_Fx = obj.Fx;
      obj.aero_Fy = obj.Fy;
      obj.aero_Fz = obj.Fz;
      obj.aero_Mx = obj.Mx;
      obj.aero_My = obj.My;
      obj.aero_Mz = obj.Mz;

      %% Add in the extra forces and moment from the extra props
      obj.prop_Fx = 0.0;
      obj.prop_Fy = 0.0;
      obj.prop_Fz = 0.0;
      obj.prop_Mx = 0.0;
      obj.prop_My = 0.0;
      obj.prop_Mz = 0.0;
      
      for ii = 1:numel(obj.Prop)

        obj.Prop{ii} = obj.Prop{ii}.aero(rho, uvw, om, obj.cm_b, ders);
        
        % extra prop forces/moments
        obj.prop_Fx = obj.prop_Fx + obj.Prop{ii}.Fx;
        obj.prop_Fy = obj.prop_Fy + obj.Prop{ii}.Fy;
        obj.prop_Fz = obj.prop_Fz + obj.Prop{ii}.Fz;
        obj.prop_Mx = obj.prop_Mx + obj.Prop{ii}.Mx;
        obj.prop_My = obj.prop_My + obj.Prop{ii}.My;
        obj.prop_Mz = obj.prop_Mz + obj.Prop{ii}.Mz;
         
        if ders
          obj.Fx_x = obj.Fx_x + obj.Prop{ii}.Fx_x;
          obj.Fy_x = obj.Fy_x + obj.Prop{ii}.Fy_x;
          obj.Fz_x = obj.Fz_x + obj.Prop{ii}.Fz_x;
          obj.Mx_x = obj.Mx_x + obj.Prop{ii}.Mx_x;
          obj.My_x = obj.My_x + obj.Prop{ii}.My_x;
          obj.Mz_x = obj.Mz_x + obj.Prop{ii}.Mz_x;

          % Each extra prop has its own set of inputs
          % Thrust and articulation angles roll, pitch, and yaw
          %{
          obj.Fx_u = [ obj.Fx_u  obj.Prop{ii}.Fx_u ];
          obj.Fy_u = [ obj.Fy_u  obj.Prop{ii}.Fy_u ];
          obj.Fz_u = [ obj.Fz_u  obj.Prop{ii}.Fz_u ];
          obj.Mx_u = [ obj.Mx_u  obj.Prop{ii}.Mx_u ];
          obj.My_u = [ obj.My_u  obj.Prop{ii}.My_u ];
          obj.Mz_u = [ obj.Mz_u  obj.Prop{ii}.Mz_u ];
%}
          offset = (ii-1)*4+6; % offset past derivatives from WingProp and Tail
          obj.Fx_u(1,offset+1:offset+4) = obj.Prop{ii}.Fx_u; % each prop is 1x4
          obj.Fy_u(1,offset+1:offset+4) = obj.Prop{ii}.Fy_u;
          obj.Fz_u(1,offset+1:offset+4) = obj.Prop{ii}.Fz_u;
          obj.Mx_u(1,offset+1:offset+4) = obj.Prop{ii}.Mx_u;
          obj.My_u(1,offset+1:offset+4) = obj.Prop{ii}.My_u;
          obj.Mz_u(1,offset+1:offset+4) = obj.Prop{ii}.Mz_u;

        end

      end

      % add extra prop forces/moments to total forces/moments
      obj.Fx = obj.Fx + obj.prop_Fx;
      obj.Fy = obj.Fy + obj.prop_Fy;
      obj.Fz = obj.Fz + obj.prop_Fz;
      obj.Mx = obj.Mx + obj.prop_Mx;
      obj.My = obj.My + obj.prop_My;
      obj.Mz = obj.Mz + obj.prop_Mz;

      %% Forces and Moments in the wind frame
      Fb = [  obj.Fx;  obj.Fy;  obj.Fz ];
      Mb = [  obj.Mx;  obj.My;  obj.Mz ];

      alf = atan2(uvw(3),uvw(1));
      obj.L = -obj.Fz*cos(alf) + obj.Fx*sin(alf);
      obj.D = -obj.Fz*sin(alf) - obj.Fx*cos(alf);

      %% Forces and Moments in the body frame
      % Fw = obj.Ry(alf)*obj.Rz(beta)'*Fb;
      % Mw = obj.Ry(alf)*obj.Rz(beta)'*Mb;
      
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw the tiltwing aircraft
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function g_hand = draw(obj, rho, V)

      % First clear the graphics handle
      obj.g_hand = [];
      alf = atan2(V(3), V(1));

      % draw the wing and tail 
      obj.WingProp = obj.WingProp.draw(rho, V, obj.cm_b);
      obj.g_hand = [obj.g_hand obj.WingProp.Left.g_hand obj.WingProp.Right.g_hand];
      obj.Tail = obj.Tail.draw(rho, V, obj.cm_b);
      obj.g_hand = [obj.g_hand obj.Tail.Left.g_hand obj.Tail.Right.g_hand obj.Tail.Vert.g_hand];

      % if there are extra props draw them as well
      for ii = 1:numel(obj.Prop)
        obj.Prop{ii} = obj.Prop{ii}.draw(obj.cm_b, V);
        obj.g_hand = [obj.g_hand obj.Prop{ii}.g_hand];
      end

      % get the wing hinge location in the velocity frame
      h_v = obj.WingProp.h_v(obj.cm_b, alf);

      % create a sphere to mark the center 
      % of mass and hinge locations
      [Xc, Yc, Zc] = sphere();
      scale = obj.WingProp.Wing.b/50;
      Xc = scale*Xc;
      Yc = scale*Yc;
      Zc = scale*Zc;

      t_hand = surf(Xc,Yc,Zc,'facecolor','k','edgecolor','none');
      obj.g_hand = [obj.g_hand t_hand];
      t_hand = surf(Xc+h_v(1),Yc+h_v(2),Zc+h_v(3),'facecolor','b','edgecolor','none');
      obj.g_hand = [obj.g_hand t_hand];
      g_hand = obj.g_hand;

       set(gca,'Ydir','reverse')
       set(gca,'Zdir','reverse')
       grid on
       axis equal
      
    end

  end

end

