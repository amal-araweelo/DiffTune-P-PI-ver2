% define the controller here

% Inputs:
%   theta_r: load position reference
%   omega_r: motor velocity reference
%   omega_dot_r
%   u: torque command

% States include
% omega_m: Motor angular velocity
% omega_l: Load angular velocity
% theta_m: motor angular position
% theta_l: load angular position
% X = [omega_m; omega_l; theta_m; theta_l]
% Xref (ini) = [omega_m; omega_l; theta_m; theta_r]
% ud: containing motor/load angular velocity/position (4outputs)

function [ud, v] = controller(X, Xref, k_vec, theta_r_dot, theta_r_2dot, v, param, dt)
                         

    % Controller gains
    k_pos = k_vec(1);
    k_vel = k_vec(2);
    k_i = k_vec(3);

    % Parameters
    N = param(1);
    J_m = param(2);
    % tau_i = k_vel/k_i;

    % States
    omega_m = X(1);
    omega_l = X(2);
    theta_m = X(3);
    theta_l = X(4);
    theta_r = Xref;

    % Define the error
    e_pos = theta_r - theta_l;

    % P-controller (modified to use error e)
    omega_r = k_pos * e_pos + N * theta_r_dot;
    omega_r_dot = k_pos * (theta_r_dot - omega_l) + N * theta_r_2dot;
        
    % Define velocity error
    s = omega_r - omega_m;
    v_dot = k_i * s;
    v = v + v_dot * dt;


    %omega_r_integ = omega_r_integ + omega_r * dt;
   
    % PI-controller
    ud = v + s * k_vel + omega_r_dot * J_m;

end