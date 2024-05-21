function [tf, psi_del, psi_del_quart] = Find_Roll_2Side_Tf(grav, Vtotal, phi_start, phi_end, psi_des) 

% This function is used to aid in determining the time required to achieve
% the desired heading change (psi_des) given a quadratic roll approximation
% of a given form (see equations below).  The function outputs the tf
% (total maneuver time) and the actual change in heading angle using the
% specified quadratic roll profile or the quartic approximation tan of the
% heading angle
%
% Inputs:
% grav:             Gravity constant (units either in m/s^2 or ft/sec^2)
% V_total:          Scalar value (i.e., constant) or function handle with velocity
%                   function of time: V(t)
% phi_start:        angle-of-bank starting angle (units rad)
% phi_end:          angle-of-bank ending angle (units rad)
% phi_time:         time allowed to transition from starting to ending phi
%                   (units sec)
% psi_des:          desired total heading change (units rad)
%
% Outputs: 
% tf                total maneuver time required (units sec)
% del_psi:          actual heading change using quadratic roll profile
% del_psi_aprox:    actual heading change using quartic approx of tan of
%                   roll angle
% psi_delta:        Total heading change during maneuver

% Written by Mike Acheson, NASA Langley Research Center
% Dynamic Systems and Control Branch (D-316)
% michael.j.acheson@nasa.gov

% Modification History:
% 4/10/2024 (MJA): original function written.

% *************************************************************************%
% Compute the quantities for the sum of the numerators of the integral of 
% % the coeficients of quartic tangent approximation
q1 = tan(phi_end);
q2 = (phi_end-phi_start)*sec(phi_start)^2;
q3 = tan(phi_start);

% Compute time required for quartic approx of tan(phi)
tf= psi_des*Vtotal/grav/(8/15*q1+2/15*q2+q3);

% Quadratic fn_omega for 2-sided roll (e.g. zero to
% phi_max back to zero...
fn_omega   = @(s) grav/Vtotal*tan(4*(phi_start-phi_end)/tf^2*(s-tf/2).^2+phi_end);
psi_del = integral(fn_omega, 0, tf, 'ArrayValued', true);

% Quadratic approx of tan(phi)
% Compute coefficients
a = 16*(q1-q2)/tf^4;
b = -32*(q1-q2)/tf^3;
c = -4*q2/tf^2+16*(q1-q2)/tf^2;
d = 4*q2/tf;
e = q3;

fn_omega4  = @(s) grav/Vtotal*(a*s.^4+b*s.^3+c*s.^2+d.*s+e);
psi_del_quart = integral(fn_omega4, 0, tf, 'ArrayValued', true);
disp('');