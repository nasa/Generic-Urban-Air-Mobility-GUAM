function [FM FM_x FM_u] = lpc_aero_wrap(model, x, u, rho)
  % wrapper for the lift plus cruise aero function to
  % model - model class
  % x - dynamic staes [vb om] 
  % u - control inputs [ delf dela dele delr omp1-9]
  % rho - air density 

% set the control inputs 
model.del_f     = u(1);
model.del_a     = u(2);
model.del_e     = u(3);
model.del_r     = u(4);

model.om_p = zeros(1,9);
model.om_p(1:4) = u(5:8)';
model.om_p(5:8) = u(9:12)';
model.om_p(9)   = u(13);

% evaluate the aerodynamics of the aircraft and 
model.aero(rho, x(1:3), x(4:6), 1);

% Retrieve the relevant derivatives
idx_del = [1 2 4 5]; % includes flaps [ delf dela dele delr];
idx_omp  = [ 7 11 15 19 23 27 31 35 39];

F = [model.Fx; model.Fy; model.Fz];
M = [model.Mx; model.My; model.Mz];

F_v = [model.Fx_x(1) model.Fx_x(2) model.Fx_x(3);
       model.Fy_x(1) model.Fy_x(2) model.Fy_x(3);
       model.Fz_x(1) model.Fz_x(2) model.Fz_x(3)];
     
M_v = [model.Mx_x(1) model.Mx_x(2) model.Mx_x(3);
       model.My_x(1) model.My_x(2) model.My_x(3);
       model.Mz_x(1) model.Mz_x(2) model.Mz_x(3)];

F_om = [model.Fx_x(4) model.Fx_x(5) model.Fx_x(6);
        model.Fy_x(4) model.Fy_x(5) model.Fy_x(6);
        model.Fz_x(4) model.Fz_x(5) model.Fz_x(6)];

M_om = [model.Mx_x(4) model.Mx_x(5) model.Mx_x(6);
        model.My_x(4) model.My_x(5) model.My_x(6);
        model.Mz_x(4) model.Mz_x(5) model.Mz_x(6)];


F_u = [ model.Fx_u(idx_del)  model.Fx_u(idx_omp);
        model.Fy_u(idx_del)  model.Fy_u(idx_omp);
        model.Fz_u(idx_del)  model.Fz_u(idx_omp)];

M_u = [ model.Mx_u(idx_del)  model.Mx_u(idx_omp);
        model.My_u(idx_del)  model.My_u(idx_omp);
        model.Mz_u(idx_del)  model.Mz_u(idx_omp)];

FM = [F; M];
FM_x = [F_v F_om; M_v M_om];
FM_u = [F_u; M_u];

