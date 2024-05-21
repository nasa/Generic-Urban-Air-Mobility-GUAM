classdef PropellerClass

  %% PUBLIC PROPERTIES %%
  properties

    %%%%% Propeller Properties %%%%%

    coef    % propeller force and moment coefficients
    spin    % propeller spin direction
    Dp      % propeller diameter
    om_prop; % propeller speed
    T;      % propeller thrust
    dT;     % Change in propeller thrust
    Q       % propeller torque
    e_b     % thrust vector orientation in body frame

    motor_mass % motor mass

    p_b % propeller in body frame
    m_b % motor in body frame
    h_b % hinge in the body frame

    %%%%% Components Relative to Hinge %%%%%  
  
    p_h % propeller in hinge frame
    m_h % motor in the hinge frame

    %%%%% propeller orientation %%%%%

    yaw   = 0;   
    pitch = 0;
    roll  = 0;

    %%%%% Mass and Geometric Properties %%%%%

    mass % mass of motor and propeller
    I    % moment of inertia 
    cm_b % center of mass in the body frame

    %%%%% Aerodynamic Forces and Moments %%%%%

    Fx % 
    Fy % 
    Fz %  
    Mx % roll moment
    My % pitching moment
    Mz % yaw moment

    Fx_x % 
    Fy_x % 
    Fz_x %  
    Mx_x % roll moment
    My_x % pitching moment
    Mz_x % yaw moment

    Fx_u % 
    Fy_u % 
    Fz_u %  
    Mx_u % roll moment
    My_u % pitching moment
    Mz_u % yaw moment

    g_hand; % Graphics object handle

  end

  %% PRIVATE HIDDEN PROPERTIES %%
  properties (Access = private)

  end

  %% PRIVATE HIDDEN PROPERTIES %%
  properties (Access = private, Hidden = true)

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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Propeller Class Constructor
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = PropellerClass(varargin)
    
      %% check to see if the proper number of 
      %% arguments were passed
      if nargin < 6 | nargin > 7
        error('incorrect number of input arguments')
      end

      %% set propeller properties
      idx = 1;

      % get the number of propellers
      NP = numel(varargin{1}); 

      % assign the propeller properties
      obj.coef       = varargin{idx}; idx = idx+1;
      obj.spin       = varargin{idx}; idx = idx+1;
      obj.Dp         = varargin{idx}; idx = idx+1;
      obj.p_b        = varargin{idx}; idx = idx+1;
      obj.m_b        = varargin{idx}; idx = idx+1;
      obj.motor_mass = varargin{idx}; idx = idx+1;
      
      %% if a thrust vector is specified set it, 
      %% otherwise assume its aligned with the body x-axis.
      if nargin == idx 
        obj.e_b = varargin{idx}/norm(varargin{idx});
      else 
        obj.e_b = repmat([1; 0; 0],1,NP);
      end

      obj.T = 0;
      obj.dT = 0;
      obj.Q = 0;
      
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
      
      obj.Fx_u = zeros(1,4);
      obj.Fy_u = zeros(1,4);
      obj.Fz_u = zeros(1,4);
      obj.Mx_u = zeros(1,4);
      obj.My_u = zeros(1,4);
      obj.Mz_u = zeros(1,4);

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the propeller hinge location
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = set.h_b(obj,value)

      % set the relative distances of the prop and the motor
      obj.m_h = obj.m_b - value;
      obj.p_h = obj.p_b - value;

      % set the hinge location
      obj.h_b = value;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propeller coordinates in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.m_b(obj)

      % check to see if there is a hinge defined
      if ~isempty(obj.h_b)
        r = obj.h_b + ...
            obj.Rx(obj.roll)*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.m_h;
      else
        r = obj.m_b;
      end
      
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propeller coordinates in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.p_b(obj)

      % check to see if there is a hinge defined
      if ~isempty(obj.h_b)
        r = obj.h_b + ...
            obj.Rx(obj.roll)*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.p_h;
      else
        r = obj.p_b;
      end

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% motor relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = m_c(obj, cm_b)

      r = obj.m_b - cm_b;

    end
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% motor coordinates in the velocity frame
    %% NOTE: sideslip angle not considered
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = m_v(obj, cm_b, alf)

      m_c = obj.m_c(cm_b);
      r = obj.Ry(alf)*m_c;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propeller relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = p_c(obj, cm_b)

      r = obj.p_b - cm_b;

    end
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propeller coordinates in the velocity frame
    %% NOTE: sideslip angle not considered
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = p_v(obj, cm_b, alf)

      p_c = obj.p_c(cm_b);
      r = obj.Ry(alf)*p_c;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = h_c(obj, cm_b)

      r = obj.h_b - cm_b;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge coordinates in the velocity frame
    %% NOTE: sideslip angle not considered
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = h_v(obj, cm_b, alf)

      h_c = obj.h_c(cm_b);
      r = obj.Ry(alf)*h_c;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% thrust vector in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = T_b(obj)

      r = obj.T*obj.Rx(obj.roll)*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.e_b;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% thrust vector in the velocity frame
    %% NOTE: sideslip angle not considered
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = T_v(obj,alf)

      r = obj.Ry(alf)*obj.T_b;

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the center of mass in the body frame
    %% NOTE: here we only consider the motor mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.cm_b(obj)

      r = obj.m_b;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the mass 
    %% NOTE: here we only consider the motor mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.mass(obj)

      r = obj.motor_mass;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get the inertia matrix
    %% NOTE: here we treat the motor as a point mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function r = get.I(obj)

      r = zeros(3); % treat as point mass for now

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% propeller aero forces
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = aero(obj, rho, uvw, om, cm_b, ders)

      % get the propeller location in the velocity frame
      p_c = obj.p_c(cm_b);

      % compute the thrust vector  
      e = obj.Rx(obj.roll)*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.e_b;

      % 
      w = [p_c' e' obj.Dp rho]; 
      x = [uvw' om'];
      u = obj.om_prop;

      T_store = obj.T;
      [T, T_x, T_om] = prop_thrust(x, u, w, obj.coef(:,1)',ders);
      [Q, Q_x, Q_om] = prop_torque(x, u, w, obj.coef(:,2)',ders);

      obj.T = T;
      if T > T_store
          obj.dT = 1;
      elseif T < T_store
          obj.dT = -1;
      else
          obj.dT = 0;
      end
      obj.Q = Q;

      F = obj.T*e;
      M = -obj.hat(F)*p_c  + obj.spin*Q*e;

      obj.Fx = F(1);
      obj.Fy = F(2);
      obj.Fz = F(3);
      obj.Mx = M(1);
      obj.My = M(2);
      obj.Mz = M(3);

      if ders
        %% Derivatives
        Sx = [0 0 0; 0 0 -1; 0 1 0];
        e_roll = Sx*obj.Rx(obj.roll)*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.e_b;

        Sy = [0 0 1; 0 0 0; -1 0 0];
        e_pitch = obj.Rx(obj.roll)*Sy*obj.Ry(obj.pitch)*obj.Rz(obj.yaw)*obj.e_b;

        Sz = [0 -1 0; 1 0 0; 0 0 0];
        e_yaw = obj.Rx(obj.roll)*obj.Ry(obj.pitch)*Sz*obj.Rz(obj.yaw)*obj.e_b;

        F_T     = e;
        F_om    = F_T*T_om;
        F_roll  = obj.T*e_roll;
        F_pitch = obj.T*e_pitch;
        F_yaw   = obj.T*e_yaw;

        M_T     = -obj.hat(F_T)*p_c;
        M_Q     = obj.spin*e;
        M_om    = M_T*T_om + M_Q*Q_om;
        M_roll  = -obj.hat(F_roll)*p_c;
        M_pitch = -obj.hat(F_pitch)*p_c;
        M_yaw   = -obj.hat(F_yaw)*p_c;

        obj.Fx_x = F_T(1)*T_x;
        obj.Fy_x = F_T(2)*T_x;
        obj.Fz_x = F_T(3)*T_x;
        obj.Mx_x = M_T(1)*T_x + M_Q(1)*Q_x;
        obj.My_x = M_T(2)*T_x + M_Q(2)*Q_x;
        obj.Mz_x = M_T(3)*T_x + M_Q(3)*Q_x;
    
        obj.Fx_u = [ F_om(1) F_roll(1) F_pitch(1) F_yaw(1)];
        obj.Fy_u = [ F_om(2) F_roll(2) F_pitch(2) F_yaw(2)];
        obj.Fz_u = [ F_om(3) F_roll(3) F_pitch(3) F_yaw(3)];
        obj.Mx_u = [ M_om(1) M_roll(1) M_pitch(1) M_yaw(1)];
        obj.My_u = [ M_om(2) M_roll(2) M_pitch(2) M_yaw(2)];
        obj.Mz_u = [ M_om(3) M_roll(3) M_pitch(3) M_yaw(3)];
      end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Draw the Propeller tube
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = draw(obj, cm_b, V)

      alf = atan2(V(3), V(1));

      p_v = obj.p_v(cm_b, alf);

      %% Draw a ring in the YZ plane
      nT = 100;
      theta = 0:2*pi/nT:2*pi;
      XT = zeros(size(theta));
      YT = obj.Dp/2*cos(theta);
      ZT = obj.Dp/2*sin(theta);

      v = cross([1; 0;0],obj.e_b);
      s = norm(v);
      c = obj.e_b'*[1; 0; 0];
      if c == -1
        R = diag([-1 -1 1]);
      else
        R = eye(3)+obj.hat(v)+obj.hat(v)^2*(1/(1+c));
      end

      %% rotate the ring into the propeller frame
      prop = obj.Ry(alf) * ...
             obj.Rx(obj.roll) * ...
             obj.Ry(obj.pitch) * ...
             obj.Rz(obj.yaw) * ...
             R*[XT; YT; ZT];
      tArrow = 0.037*obj.T*obj.Ry(alf) * ...
             obj.Rx(obj.roll) * ...
             obj.Ry(obj.pitch) * ...
             obj.Rz(obj.yaw) * ...
             obj.e_b;

      %% place the ring at the propeller center
      prop = prop + p_v;

      
      t_hand = plot3(prop(1,:), prop(2,:), prop(3,:),'k','linewidth',2);
      obj.g_hand = [obj.g_hand t_hand];
      if obj.dT == 1
        t_hand = mArrow3(p_v, p_v+tArrow,'color','red','stemWidth',0.25);
        obj.g_hand = [obj.g_hand t_hand];
      elseif obj.dT == -1
        t_hand = mArrow3(p_v, p_v+tArrow,'color','blue','stemWidth',0.25);
        obj.g_hand = [obj.g_hand t_hand];
      else 
        t_hand = mArrow3(p_v, p_v+tArrow,'color','green','stemWidth',0.25);
        obj.g_hand = [obj.g_hand t_hand];
      end
      %mArrow3(p_v, p_v+tArrow,'color','red','stemWidth',0.025);

    end

  end
end
   
