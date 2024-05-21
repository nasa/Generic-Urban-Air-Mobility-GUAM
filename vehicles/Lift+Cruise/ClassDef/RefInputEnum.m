classdef RefInputEnum < Simulink.IntEnumType
  enumeration
    FOUR_RAMP(1)
    ONE_RAMP(2)
    TIMESERIES(3)
    BEZIER(4)
    DEFAULT(5)
  end
end
