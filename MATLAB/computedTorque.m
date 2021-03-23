function [ T, qdd_ref, error] = computedTorque(t, M_est, C_est, G, Theta, Omega, qd_ref, trajS ,int_error)%,u)
%COMPUTEDTORQUE Implements computed torque controller
%   Inputs: angle, velocity, reference velocity, trajectory functions
%   (struct)
%   Outputs: torque, reference acceleration;

tmax = trajS.tmax;

if (t >= tmax)
    t = tmax; %freewheeling but without gravity
end

%60 was old omega(*2pi)
%omega_n = diag(trajS.om);
omega_n = 500*eye(4);
Kd = 2*omega_n;
Kp = omega_n.^2;
%Ki = eye(4)*omega_n;

% if (mod(t,tmax) ~= 0) || (t == 0)
%     t = mod(t,tmax);
% else
%     t = tmax;
% end

% if (t >= .78)
%    pause()
% end

q_des = trajS.f(t);
qd_des = trajS.d(t);
qdd_des = trajS.dd(t);
%qdd_des = u(1:4);

qdd_ref = qdd_des - Kd*(Omega - qd_des) - Kp*(Theta - q_des);% - Ki*int_error;
T = M_est*qdd_ref + C_est*qd_ref + G; %adjust for mass, coriolis, gravity
%T = qdd_des;
error = (Theta-q_des);
end

