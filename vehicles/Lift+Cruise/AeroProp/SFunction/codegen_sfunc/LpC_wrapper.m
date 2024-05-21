function [FM, FM_aero, FM_prop, Prop, FM_x, FM_u] = LpC_wrapper(rho, V_b, om_b, om_prop, surf, ders)
% tiltwing - tiltwing class object
% rho - atmosphere density
% V_b - body Velocity
% om_b - body Angular Rates
% om_props - prop speeds (1 x 9), rad/s
% surf - surface deflections (aileron, flap, elevator, rudder), rad
% ders - flag to control calculation of derivatives

global tiltwing;

%ders = true;

tiltwing.om_p = om_prop; % extra props
tiltwing.del_a = surf(1); % aileron
tiltwing.del_f = surf(2); % flap
tiltwing.del_e = surf(3); % elevator
tiltwing.del_r = surf(4);

tiltwing = tiltwing.aero(rho, V_b, om_b, ders);

Fb = tiltwing.total_Fb();
Mb = tiltwing.total_Mb();
FM = [Fb; Mb];

F_x = [tiltwing.Fx_x; tiltwing.Fy_x; tiltwing.Fz_x];
M_x = [tiltwing.Mx_x; tiltwing.My_x; tiltwing.Mz_x];
FM_x = [F_x; M_x];

F_u = [tiltwing.Fx_u; tiltwing.Fy_u; tiltwing.Fz_u];
M_u = [tiltwing.Mx_u; tiltwing.My_u; tiltwing.Mz_u];
FM_u = [F_u; M_u];

FM_aero = [tiltwing.aero_Fb(); tiltwing.aero_Mb];
FM_prop = [tiltwing.prop_Fb(); tiltwing.prop_Mb];

Prop = [tiltwing.Qp; tiltwing.Tp]; % torque, thrust

end

