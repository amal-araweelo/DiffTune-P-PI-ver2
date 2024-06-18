% This function defines how the sensitivity propgation runs at each sample
% time

% Inputs
% sensitivity: dxdtheta
% X: actual state, [omega_m; omega_l; theta_m; theta_l]
% Xref: desired/reference state, [omega_m; omega_l; theta_m; theta_r]
% u: torque command (output of controller, input to system)
% J_m: moment of inertia (from motor mechanical parameters)
% theta: controller gains, k_vec = [k1; k2; k_pos]
% dt: sample time (taken from simulation parameter)

% Outputs
% dXdphi: calculated sensitivity
% dudphi: calculated sensitivity

function [dXdphi,dudphi] = sensitivityComputation(dxdtheta_current, X, Xref, theta_r_dot, theta_r_2dot, v, u, param, k_vec, dt)

% Evaluate the Jacobians
dfdX = grad_f_X_fcn(X, dt, u, param(1), param(2), param(3), param(4), param(5), param(6), param(7), param(8), param(9));
dfdX = full(dfdX);    % full() converts sparse matrix to full matrix


dfdu = grad_f_u_fcn(X, dt, u, param(1), param(2), param(3), param(4), param(5), param(6), param(7), param(8), param(9));
dfdu = full(dfdu);

dhdX = grad_h_X_fcn(X, Xref, k_vec, theta_r_dot, theta_r_2dot, v, param(1), param(2), dt);
dhdX = full(dhdX);

dhdtheta = grad_h_theta_fcn(X, Xref, k_vec, theta_r_dot, theta_r_2dot, v, param(1), param(2), dt);
dhdtheta = full(dhdtheta);

% Assemble the Jacobians to compute the sensitivity
dXdphi = (dfdX + dfdu * dhdX) * dxdtheta_current + dfdu * dhdtheta;
dudphi = dhdX * dxdtheta_current + dhdtheta;

end
