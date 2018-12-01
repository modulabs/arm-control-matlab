function tau_m = control_rigid_passivity(robot, x_des, x)

n = robot.rtb.n;
Kp = diag([1200, 1200, 2200, 500, 500, 500]);
Kd = diag([20, 20, 20, 20, 20, 20]);
Lambda = diag([120, 120, 220, 10, 10, 10]);

q = x(1:n);
q_dot = x(n+1:2*n);
q_des = x_des(1:n);
q_dot_des = x_des(n+1:2*n);
q_ddot_des = x_des(2*n+1:3*n);

%
q_tilda = q_des - q;
q_dot_tilda = q_dot_des - q_dot;
q_ddot_ref = q_ddot_des + Lambda *q_dot_tilda;
q_dot_ref = q_dot_des + Lambda * q_tilda;

% dynamics with virtual reference trajectory
Mq_Cq_g = rne_modified_dh(robot.rtb, q', q_dot', q_dot_ref', q_ddot_ref')';

tau_m = Mq_Cq_g + Kp*(q_des - q) + Kd*(q_dot_des - q_dot);