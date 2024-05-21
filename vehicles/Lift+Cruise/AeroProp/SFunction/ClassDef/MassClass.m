classdef MassClass

  %% PUBLIC PROPERTIES %%
  properties

    %%%%% Mass and Geometric Properties %%%%%

    mass % mass
    I    % moment of inertia
    cm_b % center of mass in the body frame

  end

  %% PUBLIC METHODS %%
  methods 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constructor function for the tiltwing class
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function obj = MassClass(varargin)

    % set up the mass object
    % varargin : 1 - mass
    %            2 - inertia matrix
    %            3 - center of mass in the body frame

      % set the mass properties
      if nargin == 3
        obj.mass  = varargin{1};
        obj.I     = varargin{2};
        obj.cm_b  = varargin{3};
      else 
        error('incorrect number of input arguments')
      end

    end

  end

end

