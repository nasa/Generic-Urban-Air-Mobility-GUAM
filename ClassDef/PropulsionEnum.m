classdef PropulsionEnum < Simulink.IntEnumType
  enumeration
    None(1)
    FirstOrder(2)
    SecondOrder(3) 
    FirstOrderFailProp(4)
  end
end
