classdef SemiWingPropClass %#codegen
  properties

    %%%%%% Airfoil Properties %%%%%%%

    airfoil
    aero_coefs_pp
    
    %%%%%% geometry properties %%%%%%%%

    b         % Span of wing
    c_root    % cord length at the root
    c_tip     % cord length at the tip
    y_flap    % span location of the flaps
    y_aileron % span location of aileron
    D_prop    % propeller diamters
    C_prop    % propeller center location
    prop_coef % 
    spin      % propeller spin direction
    Ct_prop   % propeller thrust coefficient 
    Cq_prop   % propeller power coefficient
    
    %%%%%% control surface deflection angle %%%%%%

    del_f = 0; % flap deflection angle
    del_a = 0; % aileron deflection angle
    tau = 0.6; % flap/aileron effectiveness

    %%%%%% Strip theory Properties %%%%%%%%%%

    strip_width % width of strip
    NS      % number of strips 
    NP = 0; % number of props
    hasProp logical
    yc      % wing y-location of the center of each strip
    cc      % average chord length of each strip
    A       % Disk area of each propeller [ 1 x NP]
    Ak      % Area of the non-overlapped regions [ 1 x NP]
    Al      % Area of the overlapped regions [1 x Num overlaps]
    Si      % Area of the wing strips behind the propellers [NS x 1]
    Aki     % Area of the non-overlapped proppeller strip [NS x 1]
    Ali     % Area of the overlapped propeller strip [ NS x 1]

    Tki     % Thrust of each slice of non overlapped area
    Tli     % Thrust at each slice of overlapped area

    k_idx   % non-overlapped region index
    l_idx   % overlapped region index

    flap_idx logical   % index of flap location
    aileron_idx logical % index of aileron location
    deli  % flap/aileron deflection
    
    %%%%%% Left/Right wing indications %%%%%%%

    sgn   % Left: -1, Right: 1
    gamma=0; % wing dihedral 

    %%%%%% quarter chord and hinge informations %%%%%%

    h_b  = [0 0 0]'; % hinge coordinates in the body frame
    c4_h = [0 0 0]'; % quarter chord coordinates in the hinge frame
    c4_b = [0 0 0]'; % quarter chord coordinates in the body frame

    %%%%%% Thrust and wing Tilt inputs %%%%%%%%%

    om_prop    % propeller motor speeds
    e_b        % propeller orientation in the body frame

    Thrust     % Thrust produced by each propeller [1 x NP]
    Torque     % Torque produced by each propeller [1 x NP]
    tilt_angle = 0 % Wing tilt angle [rad]

    %%%%%% Aerodynamic Forces and Moment %%%%%%%

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

    Fx_u  
    Fy_u  
    Fz_u  
    Mx_u 
    My_u 
    Mz_u 

    Li   % Lift Force at each strip [ Li Lwi Lki Lli]
    Di   % Drag Force at each strip [ Di Dwi Dki Dli]
    alfi % angle of attack at each strip

    g_hand; %Graphics handle

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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% strip theory set up
    %% this function is still a bit of a mess 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = SetupStrips(obj)

      % might move this outside somewhere
      NS = obj.NS;

      %% make local copies of everything
      c_root = obj.c_root;
      c_tip  = obj.c_tip;
      b      = obj.b;
      D_prop = obj.D_prop;
      C_prop = obj.C_prop;

      %%%% Define propeller properties %%%%

      % find the length of the overlapped regions
      l = (C_prop(1:end-1)+D_prop(1:end-1)/2) - (C_prop(2:end)-D_prop(2:end)/2);

      Np = numel(D_prop);   % number of props per wing

      NA = Np*2-1;      % number of Areas (includes overlap areas)

      A = pi*D_prop.^2/4;  % propeller disk areas
      obj.A = A;

      %% Produce some errors if things are not right
      % if numel(D_prop)~=Np
      %   disp('Error: prop diameters'); 
      %   return; 
      % end
      % if numel(l)~=Np-1 
      %   disp('Error: prop overlap'); 
      %   return; 
      % end
      % if (C_prop(1)-D_prop(1)/2)<0
      %   disp('Error: prop fuselage clearance'); 
      %   return; 
      % end

      %%% setup the wing strips %%%
      % first find the max distance from the root of the wing, either
      % the wing tip or the far edge of an over hanging prop 
      ymax = max([ b C_prop(end)+D_prop(end)/2 ]);

      % width of each strip
      dy = ymax/NS;   

      % y points across the wing and props
      y = (0:dy:ymax)';       

      % y point of just the wing
      yw = y(y<=b);

      % chord length at each point 
      cw = c_root - yw/b*(c_root - c_tip); 

      % Area of each strip 
      Sw = dy*(cw(1:end-1)+cw(2:end))/2; 

      % tack on zeros if the propeller over hangs
      ci = [cw; zeros(sum(y>b),1)];
      Si = [Sw; zeros(sum(y>b),1)];

      % From the propeller properties get section areas and distribution  
      R1 = D_prop(1:end-1)/2;
      R2 = D_prop(2:end)/2;

      d = R1 + R2 - l;
      d1 = (d.^2+R1.^2-R2.^2) ./ (2*d);
      d2 = d-d1;

      a   = real( sqrt(R1.^2 - d1.^2) );
      th1 = real( 2*acos(d1./R1) );
      th2 = real( 2*acos(d2./R2) );
      A1 = 0.5*R1.^2.*th1 - a.*d1;
      A2 = 0.5*R2.^2.*th2 - a.*d2;

      % Area of the overlapped region
      Al = A1+A2;

      AL = [0 Al 0];  % wrap in zeros
      L  = [0 l 0];   % wrap in zeros 
      
      % area of the non-overlapped regions
      Ak = A - AL(1:end-1) - AL(2:end);

      % Compute the region areas and pack into an array
      odd = 1:2:NA;
      even = 2:2:NA;


      % figure out the spacing between each region
      dY = zeros(1,NA);
      dY(odd) = D_prop ...
                - L(1:end-1).*(L(1:end-1)>0) ...
                - L(2:end).*(L(2:end)>0);

      dY(even) = abs(l);

      Y = cumsum(dY) + (C_prop(1) - D_prop(1)/2);

      % Disk intersection points
      Di = [C_prop(1:end-1)+d1 Y(end)];

      % plot non-overlap region and estimate the area 
      zt = zeros(size(y));
      k_idx = zeros(size(y));
      for i = Np:-1:1
        idx = y<Di(i);
        zt( idx ) = real( sqrt(complex(D_prop(i)^2/4 - (y(idx) - C_prop(i)).^2)) );
        k_idx( idx ) = i;
      end
      k_idx = k_idx.*(zt>0);
      k_idx(end) = [];

      % plot overlap region and estimate the area 
      zl = zeros(size(y));
      l_idx = zeros(size(y));
      for i = 1:Np-1

        j = y<Di(i) & y>(Di(i)-d1(i));
        k = y>Di(i) & y<(Di(i)+d2(i));

        zl(j) = real( sqrt(complex(D_prop(i+1)^2/4 - (y(j) - C_prop(i+1)).^2)) );
        zl(k) = real( sqrt(complex(D_prop(i)^2/4 - (y(k) - C_prop(i)).^2)) );

        l_idx(j) = i;
        l_idx(k) = i;
      end
      l_idx = l_idx.*(zl>0);
      l_idx(end) = [];

      % Estimate the area of the overlapped region using traps
      Ali = (zl(1:end-1) + zl(2:end))*dy;

      % Estimate the area of the non-overlapped region using traps
      zp = zt-zl;
      Aki = (zp(1:end-1) + zp(2:end))*dy;


      %% Save all the setup parameter to be used later

      obj.yc = y(2:end)-diff(y)/2;  % y distance from root to center of strip
      obj.cc =( ci(1:end-1) + ci(2:end) )/2; % wing strip center cord lengths
    
      %obj.A = A;   % Disk area of each prop
      obj.Ak = Ak; % propeller non-overlapped areas
      obj.Al = Al; % propeller overlapped areas

      obj.Si = Si; % wing strip areas
      obj.Aki = Aki; % propeller non overlapped strip areas
      obj.Ali = Ali; % propeller over lapped strip areas

      obj.k_idx = k_idx;
      obj.l_idx = l_idx;

      obj.Tki = zeros(size(k_idx));
      obj.Tli = zeros(size(l_idx));

      
      %% Save the flap and aileron indices
      obj.flap_idx = obj.yc >= obj.y_flap(1) & obj.yc <= obj.y_flap(2);
      obj.aileron_idx = obj.yc >= obj.y_aileron(1) & obj.yc <= obj.y_aileron(2);

      % mark wing strips with out defelcting surfaces
      obj.deli(~(obj.flap_idx | obj.aileron_idx)) = 255;

    end
    
  end

  %% PUBLIC METHODS
  methods

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for the semi wing span class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = SemiWingPropClass(varargin)
    % set up the wingProp 
    % varargin : 1 - Left/Right
    %            2 - airfoil
    %            3 - aero coefficients pp
    %            4 - span 
    %            5 - c, or if uniformly tapered [c_root c_tip]
    %            6 - dihedral angle
    %            7 - y_flap
    %            8 - y_aileron
    %            9 - c4_b wing quarter chord
    %            10 - D_prop
    %            11 - C_prop

%      obj.strip_width = .01;
      obj.strip_width = .2;
      obj.NS = floor(varargin{4}/obj.strip_width);

      % initialize all strip storage 
      obj.yc          = zeros(obj.NS,1);
      obj.Si          = zeros(obj.NS,1);
      obj.Aki         = zeros(obj.NS,1);
      obj.Ali         = zeros(obj.NS,1);
      obj.k_idx       = zeros(obj.NS,1);
      obj.l_idx       = zeros(obj.NS,1);
      obj.Tki         = zeros(obj.NS,1);
      obj.Tli         = zeros(obj.NS,1);
      obj.flap_idx    = zeros(obj.NS,1);
      obj.aileron_idx = zeros(obj.NS,1);
      obj.deli        = zeros(obj.NS,1);
      obj.Li          = zeros(obj.NS,1);
      obj.Di          = zeros(obj.NS,1);
      obj.alfi        = zeros(obj.NS,1);
      
      obj.hasProp = false;
      
      if nargin == 9 || nargin == 13

        % start input index counter
        idx = 1;

        % check left or right side
        if strcmp(varargin{idx},'Left')
          obj.sgn = -1;
        elseif strcmp(varargin{idx},'Right')
          obj.sgn = 1;
        else
          error('first argument must specify Left of Right');
        end
        idx = idx+1;

        % assign wing parameters
        obj.airfoil       = varargin{idx}; idx = idx+1;
        obj.aero_coefs_pp = varargin{idx}; idx = idx+1;
        obj.b             = varargin{idx}; idx = idx+1;
        chord             = varargin{idx}; idx = idx+1;
        obj.gamma         = varargin{idx}; idx = idx+1;
        obj.y_flap        = varargin{idx}; idx = idx+1;
        obj.y_aileron     = varargin{idx}; idx = idx+1;
        obj.h_b           = varargin{idx}; idx = idx+1;

        % assign propeller properties
        if nargin > idx
          obj.prop_coef   = varargin{idx}; idx = idx+1;
          obj.spin        = varargin{idx}; idx = idx+1;
          obj.D_prop      = varargin{idx}; idx = idx+1;
          obj.C_prop      = varargin{idx}; idx = idx+1;

          % check the number of diameters and centers specified
          if numel(obj.D_prop) ~= numel(obj.C_prop)
            error('size of D_prop and C_prop must be consistent');
          end
          obj.NP = numel(obj.D_prop);
          obj.hasProp = true;

          % break out the coefficients
          obj.Ct_prop = zeros(3,obj.NP);
          obj.Cq_prop = zeros(3,obj.NP);
          for ii = 1:obj.NP
            obj.Ct_prop = obj.prop_coef{ii}(:,1);
            obj.Cq_prop = obj.prop_coef{ii}(:,2);
          end

        else  

          % if no propellers are specified then define one prop that is 
          % centered on the wing and who's dimater is equal to the span.
          % Setting the thrust equal to zero will cause the "ghost" propeller
          % to have no effect on the aerodynamics.
          obj.prop_coef = zeros(3,2);
          obj.spin = 1;
          obj.D_prop = obj.b;
          obj.C_prop = obj.b/2;
          obj.NP = 0;
          obj.hasProp = false;
          obj.Ct_prop = zeros(3,1);
          obj.Cq_prop = zeros(3,1);

        end

        % assign the chord length
        if numel(chord) == 1
          obj.c_root  = chord;
          obj.c_tip   = chord;
        elseif numel(chord) == 2
          obj.c_root = chord(1);
          obj.c_tip  = chord(2);
        else 
          error('Too many arguments in chord length') 
        end

        % assign the quarter chord position in the body frame. When
        % setting up the wing we assume that if the wing rotates, that
        % it is hinged at the quarter chord and the initial tilt angle
        % is zero.
        obj.c4_h  = [0 0 0]';
        obj.tilt_angle     = 0;

        % Finally set up the strips
        obj = SetupStrips(obj);
%        if obj.NP  > 0 
        if obj.hasProp
          obj.Thrust = zeros(1,obj.NP); % initialize thrust at zero
          obj.Torque = zeros(1,obj.NP);
          obj.om_prop = zeros(obj.NP,1);
          obj.e_b = zeros(3,obj.NP);
        else
          obj.Thrust = zeros(1,1);
          obj.Torque = zeros(1,1);
          obj.om_prop = zeros(1,1);
          obj.e_b = zeros(3,1);
        end

        obj.A = pi*obj.D_prop.^2/4;  % propeller disk areas
      else
        obj.k_idx;
        obj.l_idx;
        obj.A = 0;
        error('Incorrect inputs');
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the speed of propeller
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = set.om_prop(obj, value)
      if numel(value) ~= obj.NP && obj.hasProp 
        error('incorrect number of motor speed values specified')
      else
        obj.om_prop = value;
      end

    end


     

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the thrust of each prop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = set.Thrust(obj, value)
      if (numel(value) ~= obj.NP) && obj.hasProp 
        error('incorrect number of thrust values specified')
      else
        obj.Thrust = value;

        idx = obj.k_idx > 0;

        % % adjust the thrust at the strip locations
        T = obj.Thrust(1,obj.k_idx(idx)); 
        A = obj.A(1,obj.k_idx(idx)); 
        Aki = obj.Aki(idx);
        obj.Tki(idx) = T(:).*Aki(:)./A(:);

        % % Overlapped Thrust
        idx = obj.l_idx > 0;

        Tl = obj.OverlapThrust();

        Tl = Tl(obj.l_idx(idx)); 
        Al = obj.Al(obj.l_idx(idx)); 
        Ali = obj.Ali(idx);

        obj.Tli(idx) = Tl(:).*Ali(:)./Al(:);

      end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the flap deflection
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = set.del_f(obj, value)

      % get the aileron values where they overlap with flaps
      aileron = obj.flap_idx.*obj.aileron_idx.*obj.del_a;

      % set the flap angle
      obj.deli(obj.flap_idx) = value - obj.sgn*aileron(obj.flap_idx);

      % finally set the flapp value
      obj.del_f = value;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set the aileron deflection
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = set.del_a(obj, value)

      % get the flap values where they overlap with aileron
      flap = obj.flap_idx.*obj.aileron_idx.*obj.del_f;

      % set the aileron angle
      obj.deli(obj.aileron_idx) = flap(obj.aileron_idx) - obj.sgn*value;

      % finally set the flapp value
      obj.del_a = value;

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% quarter chord coordinates in the body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = get.c4_b(obj)
      r = obj.h_b + obj.Ry(obj.tilt_angle)*obj.c4_h;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% quarter chord relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = c4_c(obj, cm_b)
      r = obj.c4_b - cm_b;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% quarter chord coordinates in the velocity frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = c4_v(obj, cm_b, alf)
      c4_c = obj.c4_c(cm_b);
      r = obj.Ry(alf)*c4_c;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge relative to the center of mass
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = h_c(obj, cm_b)
      r = obj.h_b - cm_b;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% hinge coordinates in the velocity frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function r = h_v(obj, cm_b, alf)
      h_c = obj.h_c(cm_b);
      r = obj.Ry(alf)*h_c;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Thrust at the overlapped regions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [Tl] = OverlapThrust(obj)

      Tbar = (obj.Thrust(1:end-1)+obj.Thrust(2:end)).*...
      				((obj.A(1:end-1)+obj.A(2:end)-obj.Al)./...
              (obj.A(1:end-1)+obj.A(2:end))).^(1/3);

      
      Tl = Tbar...
      		 - obj.Thrust(1:end-1).*(obj.A(1:end-1)-obj.Al)./obj.A(1:end-1)...
      		 - obj.Thrust(2:end).*(obj.A(2:end)-obj.Al)./obj.A(2:end);
    end
      

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% induced velocity produced at the propeller disk
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [w] = InducedVelocity(obj, v, T, A, rho)

      w = 0.5*( -v + sqrt( v.^2 + 2*T./(A*rho) ) );

      % handles sections with zero area
      w(isnan(w)) = 0;

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute the slipstream properties based on the 
    % freestream conditions and the propeller thrust
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [v_s, alf_s, q_s, cTs] = Slipstream(obj, rho, v, alf, T, A)

      v_perp = v.*cos(alf);
      w = obj.InducedVelocity(v_perp,T,A,rho);
      v_s = sqrt( (2*w + v.*cos(alf)).^2 + (v.*sin(alf)).^2 );
      alf_s = atan2( v.*sin(alf), 2*w + v.*cos(alf) );
      q_s = 0.5*rho*v_s.^2;
      cTs = T./(A.*q_s);

      % handle NaN's do to sections with no prop area in front
      cTs(isnan(cTs)) = 0;

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% aerodynamic forces and moment in the velocity frame %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = aero(obj, rho, uvw, om, cm_b, ders)
      
      % wing quarter chord relative to the center of mass
      c4_c = obj.c4_c(cm_b);

      % For now we do not deal with dihedral well and assume that the 
      % wing rotates about the quarter chord
      b = obj.sgn*(obj.Rx(-obj.sgn*obj.gamma)*[0*obj.yc obj.yc 0*obj.yc]')'...
          + repmat(c4_c',obj.NS,1); 

      w = [ b obj.Aki obj.Ali obj.Si repmat(rho,obj.NS,1) repmat(-obj.sgn*obj.gamma,obj.NS,1)];
      
      % This needs to happen before the Force and moment 
      % computations in order to take into account the 
      % slip stream effect on the wings
      if obj.hasProp 
        
        % get prop positions
        % NEED TO ADD AN OFFSET FROM C/4
        b_prop = obj.sgn*[0*obj.C_prop obj.C_prop  0*obj.C_prop] + repmat(c4_c',obj.NP,1);

        % get prop orientation
        % NEGLECTS THE DERVATIVE OF THRUST AND TORQUE WITH RESPECT TO THE
        % TILT-ANGLE
        e_prop = repmat([cos(obj.tilt_angle) 0 -sin(obj.tilt_angle)],obj.NP,1);
        
        w_prop = [b_prop e_prop obj.D_prop repmat(rho, obj.NP,1)];
        u_prop = obj.om_prop';
        x_prop = repmat([uvw' om'], obj.NP,1);


        [T, T_x, T_om] = prop_thrust(x_prop, u_prop, w_prop, obj.Ct_prop',ders);
        [Q, Q_x, Q_om] = prop_torque(x_prop, u_prop, w_prop, obj.Cq_prop',ders);

        obj.Thrust = T;
        obj.Torque = Q;

      else
        e_prop = zeros(1,3);
        Q = zeros(1,1);
        Q_x = zeros(1,6);
        Q_om = zeros(1,1);
        T_x = zeros(1,6);
        T_om = zeros(1,1);
      end 

      %% build the input vector
      u = [obj.Tki obj.Tli obj.deli repmat(obj.tilt_angle, obj.NS, 1)];

      %% build the state vector
      x = repmat([uvw' om'], obj.NS,1);

      %% compute the stripwise lift drag and pitching moment
      [FMi, FMi_x, FMi_u] = FM_body(x,u,w,obj.aero_coefs_pp,ders);

      Fxi = FMi(:,1);
      Fyi = FMi(:,2);
      Fzi = FMi(:,3);
      Mxi = FMi(:,4);
      Myi = FMi(:,5);
      Mzi = FMi(:,6);

      % save outputs to the object
      obj.Fx = sum(Fxi);
      obj.Fy = sum(Fyi);
      obj.Fz = sum(Fzi);
      obj.Mx = sum(Mxi);
      obj.My = sum(Myi);
      obj.Mz = sum(Mzi);

      % NS x 6, NS x 4
      Fxi_x = FMi_x(:,:,1); Fxi_u = FMi_u(:,:,1);
      Fyi_x = FMi_x(:,:,2); Fyi_u = FMi_u(:,:,2);
      Fzi_x = FMi_x(:,:,3); Fzi_u = FMi_u(:,:,3);
      Mxi_x = FMi_x(:,:,4); Mxi_u = FMi_u(:,:,4);
      Myi_x = FMi_x(:,:,5); Myi_u = FMi_u(:,:,5);
      Mzi_x = FMi_x(:,:,6); Mzi_u = FMi_u(:,:,6);

      if ders

        % flaps 
        dFxi_df = zeros(obj.NS,1);
        dFyi_df = zeros(obj.NS,1);
        dFzi_df = zeros(obj.NS,1);
        dMxi_df = zeros(obj.NS,1);
        dMyi_df = zeros(obj.NS,1);
        dMzi_df = zeros(obj.NS,1);

        dFxi_df(obj.flap_idx) = Fxi_u(obj.flap_idx,3);
        dFyi_df(obj.flap_idx) = Fyi_u(obj.flap_idx,3);
        dFzi_df(obj.flap_idx) = Fzi_u(obj.flap_idx,3);
        dMxi_df(obj.flap_idx) = Mxi_u(obj.flap_idx,3);
        dMyi_df(obj.flap_idx) = Myi_u(obj.flap_idx,3);
        dMzi_df(obj.flap_idx) = Mzi_u(obj.flap_idx,3);

        % ailerons
        dFxi_da = zeros(obj.NS,1);
        dFyi_da = zeros(obj.NS,1);
        dFzi_da = zeros(obj.NS,1);
        dMxi_da = zeros(obj.NS,1);
        dMyi_da = zeros(obj.NS,1);
        dMzi_da = zeros(obj.NS,1);

        dFxi_da(obj.aileron_idx) = -obj.sgn*Fxi_u(obj.aileron_idx,3);
        dFyi_da(obj.aileron_idx) = -obj.sgn*Fyi_u(obj.aileron_idx,3);
        dFzi_da(obj.aileron_idx) = -obj.sgn*Fzi_u(obj.aileron_idx,3);
        dMxi_da(obj.aileron_idx) = -obj.sgn*Mxi_u(obj.aileron_idx,3);
        dMyi_da(obj.aileron_idx) = -obj.sgn*Myi_u(obj.aileron_idx,3);
        dMzi_da(obj.aileron_idx) = -obj.sgn*Mzi_u(obj.aileron_idx,3);

        % tilt angle 
        dFxi_di = Fxi_u(:,4);
        dFyi_di = Fyi_u(:,4);
        dFzi_di = Fzi_u(:,4);
        dMxi_di = Mxi_u(:,4);
        dMyi_di = Myi_u(:,4);
        dMzi_di = Mzi_u(:,4);

        % Gather the aero surface input derivatives
        Fxi_ub = [dFxi_df dFxi_da dFxi_di]; % NS x 3
        Fyi_ub = [dFyi_df dFyi_da dFyi_di];
        Fzi_ub = [dFzi_df dFzi_da dFzi_di];
        Mxi_ub = [dMxi_df dMxi_da dMxi_di];
        Myi_ub = [dMyi_df dMyi_da dMyi_di];
        Mzi_ub = [dMzi_df dMzi_da dMzi_di];

        % add in the derivative of the Thrust with the change in state
        obj.Fx_x = sum(Fxi_x); % 1 x 6
        obj.Fy_x = sum(Fyi_x);
        obj.Fz_x = sum(Fzi_x);
        obj.Mx_x = sum(Mxi_x);
        obj.My_x = sum(Myi_x);
        obj.Mz_x = sum(Mzi_x);

        obj.Fx_u(1,obj.NP+1:end) = sum(Fxi_ub); % 1 x 3
        obj.Fy_u(1,obj.NP+1:end) = sum(Fyi_ub);
        obj.Fz_u(1,obj.NP+1:end) = sum(Fzi_ub);
        obj.Mx_u(1,obj.NP+1:end) = sum(Mxi_ub);
        obj.My_u(1,obj.NP+1:end) = sum(Myi_ub);
        obj.Mz_u(1,obj.NP+1:end) = sum(Mzi_ub);

      end

      % Get Propeller effects and Derivatives
      if obj.hasProp 

        % The forces and moments already have the 
        % contributions from the propeller thrust
        % but we need to add in the torque contribution 
        % to the moments
        ex = obj.spin.*e_prop(1:obj.NP,1);
        ey = obj.spin.*e_prop(1:obj.NP,2);
        ez = obj.spin.*e_prop(1:obj.NP,3);
        
        obj.Mx = obj.Mx + ex'*Q(1:obj.NP,1);
        obj.My = obj.My + ey'*Q(1:obj.NP,1);
        obj.Mz = obj.Mz + ez'*Q(1:obj.NP,1);


        if ders 

          %% take the rest of the derivatives here
          % thrust 
          dFxi_dT = zeros(obj.NS, obj.NP);
          dFyi_dT = zeros(obj.NS, obj.NP);
          dFzi_dT = zeros(obj.NS, obj.NP);
          dMxi_dT = zeros(obj.NS, obj.NP);
          dMyi_dT = zeros(obj.NS, obj.NP);
          dMzi_dT = zeros(obj.NS, obj.NP);
          
          % Non-overlapped regions
          for ii = 1:obj.NP
            idx = (obj.k_idx == ii);

            dFxi_dT(idx,ii) = Fxi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;
            dFyi_dT(idx,ii) = Fyi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;
            dFzi_dT(idx,ii) = Fzi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;
            dMxi_dT(idx,ii) = Mxi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;
            dMyi_dT(idx,ii) = Myi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;
            dMzi_dT(idx,ii) = Mzi_u(idx,1).*obj.Aki(idx)/obj.A(ii) ;

          end

          % Overlapped regions
          for ii = 1:length(obj.Al)
            idx = (obj.l_idx == ii);
           
            dTl_dT1 = ((obj.A(ii)+obj.A(ii+1)-obj.Al(ii)) / (obj.A(ii)+obj.A(ii+1)) )^(1/3)...
                     - (obj.A(ii)-obj.Al(ii))/obj.A(ii);

            dTl_dT2 = ((obj.A(ii)+obj.A(ii+1)-obj.Al(ii)) / (obj.A(ii)+obj.A(ii+1)) )^(1/3)...
                     - (obj.A(ii+1)-obj.Al(ii))/obj.A(ii+1);

            dTi_dTl = obj.Ali(idx)/obj.Al(ii);


            % Fx derivatives
            dFxi_dT(idx,ii) = dFxi_dT(idx,ii) + Fxi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dFxi_dT(idx,ii+1) = dFxi_dT(idx,ii+1) + Fxi_u(idx,2).*dTi_dTl.*dTl_dT2;

            % Fy derivatives
            dFyi_dT(idx,ii)   = dFyi_dT(idx,ii)   + Fyi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dFyi_dT(idx,ii+1) = dFyi_dT(idx,ii+1) + Fyi_u(idx,2).*dTi_dTl.*dTl_dT2;

            % Fz derivatives
            dFzi_dT(idx,ii)   = dFzi_dT(idx,ii)   + Fzi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dFzi_dT(idx,ii+1) = dFzi_dT(idx,ii+1) + Fzi_u(idx,2).*dTi_dTl.*dTl_dT2;

            % Mx derivatives
            dMxi_dT(idx,ii) = dMxi_dT(idx,ii) + Mxi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dMxi_dT(idx,ii+1) = dMxi_dT(idx,ii+1) + Mxi_u(idx,2).*dTi_dTl.*dTl_dT2;

            % My derivatives
            dMyi_dT(idx,ii) = dMyi_dT(idx,ii) + Myi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dMyi_dT(idx,ii+1) = dMyi_dT(idx,ii+1) + Myi_u(idx,2).*dTi_dTl.*dTl_dT2;

            % My derivatives
            dMzi_dT(idx,ii) = dMzi_dT(idx,ii) + Mzi_u(idx,2).*dTi_dTl.*dTl_dT1;
            dMzi_dT(idx,ii+1) = dMzi_dT(idx,ii+1) + Mzi_u(idx,2).*dTi_dTl.*dTl_dT2;

          end

          % derivative with respect to the propeller speeds propeller speeds
          dFx_dT = sum(dFxi_dT); % 1 x NP
          dFy_dT = sum(dFyi_dT);
          dFz_dT = sum(dFzi_dT);
          dMx_dT = sum(dMxi_dT);
          dMy_dT = sum(dMyi_dT);
          dMz_dT = sum(dMzi_dT);

          dFx_dom = dFx_dT.*T_om; % 1 x NP
          dFy_dom = dFy_dT.*T_om;
          dFz_dom = dFz_dT.*T_om;
          dMx_dom = dMx_dT.*T_om;
          dMy_dom = dMy_dT.*T_om;
          dMz_dom = dMz_dT.*T_om;


          % add in derviative of thrust with respect to the state
          obj.Fx_x = obj.Fx_x + dFx_dT*T_x; % 1 x 6
          obj.Fy_x = obj.Fy_x + dFy_dT*T_x;
          obj.Fz_x = obj.Fz_x + dFz_dT*T_x;
          
          obj.Mx_x = obj.Mx_x + dMx_dT*T_x + ex'*Q_x;
          obj.My_x = obj.My_x + dMy_dT*T_x + ey'*Q_x;
          obj.Mz_x = obj.Mz_x + dMz_dT*T_x + ez'*Q_x;

          % add in the propeller speed derivatives to 
          % the input derivatives
          obj.Fx_u(1,1:obj.NP) = dFx_dom; % 1xNP
          obj.Fy_u(1,1:obj.NP) = dFy_dom;
          obj.Fz_u(1,1:obj.NP) = dFz_dom;

          obj.Mx_u(1,1:obj.NP) = dMx_dom+ex'*Q_om; % 1xNP
          obj.My_u(1,1:obj.NP) = dMy_dom+ey'*Q_om;
          obj.Mz_u(1,1:obj.NP) = dMz_dom+ez'*Q_om;

        end % if ders

      end % if NP > 0

      % Calculatet the Lift and Drag at each Strip
      
      % Mostly used in visualizations
      alf = atan2(uvw(3),uvw(1));
      obj.Li = -Fzi*cos(alf) + Fxi*sin(alf);
      obj.Di = -Fzi*sin(alf) - Fxi*cos(alf);

    end

     
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw the wing with props
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function obj = draw(obj, rho, V, cm_b)

        % angle of attack
        alf = atan2(V(3),V(1));

        % hinge and quarter chord in velocity frame
        h_v = obj.h_v(cm_b, alf);
        c4_v = obj.c4_v(cm_b, alf);

        % Draw the wing 
        x_h = (-obj.airfoil(:,1) + 0.25) + obj.c4_h(1)/obj.c_root; 
        y_h = 0*obj.airfoil(:,1);
        z_h= -obj.airfoil(:,2);

        X_h = [x_h y_h z_h]';
        X_v = obj.Ry(alf+obj.tilt_angle)*X_h;

        nRows = numel(x_h);
        nCols = numel(obj.yc)+1;

        x_v = repmat(X_v(1,:)', 1, nCols)*diag([obj.cc; 0]) + h_v(1);
        y_v = repmat(obj.sgn*[obj.yc; obj.yc(end)]', nRows, 1)   + c4_v(2);
        z_v = repmat(X_v(3,:)', 1, nCols)*diag([obj.cc; 0]) + h_v(3);

        L = repmat([obj.Li(:,1); obj.Li(end,1)]' , nRows, 1); 

        t_hand = surf(x_v, y_v, z_v, L,'edgecolor','none');
        hand_size = length(obj.g_hand);
        obj.g_hand(hand_size+1) = t_hand;


        % if there are propellers present on the wing, draw them as well.
        % This part still needs work determining the number of 
        % propellers and which slipstream velocity belong to which
        % section.
        if obj.hasProp
          % Draw propeller

          % get the slipstream properties
          alf_w = obj.tilt_angle + alf;
          v = norm(V,2);
          [v_s, alf_s, q_s, cTs]=obj.Slipstream(rho,v,alf_w,obj.Thrust,obj.A);
           

          for ii = 1:obj.NP 
            %% Draw a ring in the YZ plane
            nT = 20;
            theta = 0:2*pi/nT:2*pi;
            Xp_ = zeros(size(theta));
            Yp_ = obj.D_prop(ii)/2*cos(theta);
            Zp_ = obj.D_prop(ii)/2*sin(theta);


            %% center of each prop 
            Xc = (obj.c_root/2 + obj.c4_h(1)+.1)*cos(alf_w) ...
                 + ( obj.c4_h(3))*sin(alf_w)...
                 + h_v(1);
            Yc = obj.c4_h(2) + obj.sgn*obj.C_prop(ii);

            Zc = -(obj.c_root/2 +obj.c4_h(1)+.1)*sin(alf_w)...
                 +(obj.c4_h(3))*cos(alf_w)...
                 + h_v(3);

            X =   (Xp_+ obj.c_root/2 + obj.c4_h(1)+.1)*cos(alf_w) ...
                 + (Zp_+ obj.c4_h(3))*sin(alf_w)...
                 + h_v(1);

            Y =  Yp_ + obj.c4_h(2) + obj.sgn*obj.C_prop(ii);

            Z = -(Xp_+ obj.c_root/2 +obj.c4_h(1)+.1)*sin(alf_w)...
                 +(Zp_+obj.c4_h(3))*cos(alf_w)...
                 + h_v(3);

            %% Draw slipstream cyclinder
            [Z_ Y_ X_] = cylinder(obj.D_prop(ii)/2);

            X_w_ = -2*X_*obj.c_root;

            X_w =   (X_w_ + obj.c_root/2 + obj.c4_h(1))*cos(alf_w-alf_s(ii)) ...
                 + (Z_+ obj.c4_h(3))*sin(alf_w-alf_s(ii))...
                 + h_v(1);
            Y_w =  Y_ + obj.c4_h(2) + obj.sgn*obj.C_prop(ii);

            Z_w = -(X_w_+ obj.c_root/2 +obj.c4_h(1))*sin(alf_w-alf_s(ii))...
                 +(Z_+obj.c4_h(3))*cos(alf_w-alf_s(ii))...
                 + h_v(3);

            %% thrust Arrow
            tCenter = [Xc; Yc; Zc];
            tArrow = 0.05*obj.Thrust(ii)*obj.Ry(alf_w)*[1; 0; 0];

            % calculate the propwash transparency 
            alpha = 0.05;
            plot3(X,Y,Z,'k','linewidth',2)
            t_hand = surf(X_w,Y_w,Z_w,'facecolor','k','edgecolor','none','facealpha',alpha)
            hand_size = length(obj.g_hand);
            obj.g_hand(hand_size+1) = t_hand;
            t_hand = mArrow3(tCenter, tCenter+tArrow,'color','red','stemWidth',0.0025);
            hand_size = length(obj.g_hand);
            obj.g_hand(hand_size+1) = t_hand;
          end
        end
          
    end

  
  end
end
