function tau_m = control_rigid_passivity(robot, x, x_des)

persistent q_int

n = robot.rtb.n;
Kp = diag([100, 100, 100, 100, 100, 100]);
Kd = diag([20, 20, 20, 20, 20, 20]);
Ki = diag([0, 0, 0, 0, 0, 0]);
q_int_max = 10000*ones(n,1);
q_int_min = -q_int_max;

q = x(1:n);
q_dot = x(n+1:2*n);
q_des = x_des(1:n);
q_dot_des = x_des(n+1:2*n);
q_ddot_des = x_des(2*n+1:3*n);

% inverse dynamics
M = robot.rtb.inertia(q');
M_qdd_c_g = robot.rtb.rne(q', q_dot', q_ddot_des');    % M(q)*q_ddot_des + c + g

% control torque
q_tilda = q_des - q;
q_dot_tilda = q_dot_des - q_dot;
if isempty(q_int)
    q_int = zeros(n,1);
else
    q_int = q_int + q_tilda;
end
    
% saturate integral
for i=1:n
    if q_int(i) > q_int_max(i)
        q_int(i) = q_int_max(i);
    elseif q_int(i) < q_int_min(i)
        q_int(i) = q_int_min(i);
    end
end

tau_m = M*(Kp*q_tilda + Kd*q_dot_tilda + Ki*q_int) + M_qdd_c_g';