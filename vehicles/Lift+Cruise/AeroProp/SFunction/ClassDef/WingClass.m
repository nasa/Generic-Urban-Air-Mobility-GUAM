classdef WingClass

  %% PUBLIC PROPERTIES %%
  properties

    %%%%%% Wing Aerodynamic Properties %%%%%%

    airfoil   % Airfoil 2-D profile (used for drawing)
    coeff     % airfoil aerodynamic coefficients  [ alf Cl Cd Cm]
    b         % wing span
    b_e       % exposed wing span
    c_root    % wing chord at the root
    c_tip     % wing chord at the tip
    gamma     % wing dihedral angle
    y_flap    % wing location of flap
    y_aileron % wing location of aileron
    c4_b      % quarter chord in the body frame

    %%%%% Mass and Geometry Properties %%%%%

    mass     % mass of the wing
    I        % Inertia Matrix of wing
    cm_b     % center of mass in the body frame

  end

  %% PUBLIC METHODS %%
  methods 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor for a wing class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = WingClass(varargin)

    % set up the wing
    % varargin : 1 - airfoil
    %            2 - coeff
    %            3 - span 
    %            4 - c, or if uniformly tapered [c_root c_tip]
    %            5 - wing dihedral angle
    %            6 - y_flap
    %            7 - y_aileron
    %            8 - c4_b wing quarter chord (tilt angle = 0)
    %            9 - mass
    %           10 - Inertia matrix
    %           11 - center of mass in body frame
  
      if nargin == 11

        obj.airfoil = varargin{1};
        obj.coeff = varargin{2};

        % assign the wing span (semispan of entire wing)
        b = varargin{3};
        if numel(b) == 1
          obj.b   = b;
          obj.b_e = b;
        elseif numel(b) == 2
          obj.b = b(1);
          obj.b_e = b(2);
        else 
          error('Too many arguments in wing span length') 
        end

        % assign the chord length
        c = varargin{4};
        if numel(c) == 1
          obj.c_root  = c;
          obj.c_tip   = c;
        elseif numel(c) == 2
          obj.c_root = c(1);
          obj.c_tip = c(2);
        else 
          error('Too many arguments in chord length') 
        end

        obj.gamma = varargin{5};
        obj.y_flap = varargin{6};
        obj.y_aileron = varargin{7};
        obj.c4_b   = varargin{8};
        obj.mass = varargin{9};
        obj.I = varargin{10};
        obj.cm_b   = varargin{11};

      else
        error('incorrect number of input arguments')
      end 
   
    end
      
  end

end
  
