classdef FlightModeEnum < Simulink.IntEnumType
  enumeration
    GLIDER(1)
    CRUISE(2) 
    TRANSITION(3)
    HOVER(4)
    VERTICAL(5)
  end
end
