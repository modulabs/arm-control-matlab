function tau_m = control_rigid_passivity(robot, state, state_des, tau_ext)

n = robot.rtb.n;
Kp = diag([100, 100, 100, 100, 100, 100]);
Kd = diag([20, 20, 20, 20, 20, 20]);
Lambda = diag([1, 1, 1, 1, 1, 1]);

q = state.q;
q_dot = state.q_dot;
q_des = state_des.q;
q_dot_des = state_des.q_dot;
q_ddot_des = state_des.q_ddot;

% inverse dynamics
M = robot.rtb.inertia(q');
C = robot.rtb.coriolis(q', q_dot');
g = robot.rtb.gravload(q')';

% tau_m = zeros(n,1);
q_tilda = q_des - q;
q_dot_tilda = q_dot_des - q_dot;
q_ddot_ref = q_ddot_des + Lambda *q_dot_tilda;
q_dot_ref = q_dot_des + Lambda * q_tilda;
tau_m = M*q_ddot_ref + C*q_dot_ref + g + Kp*(q_des - q) + Kd*(q_dot_des - q_dot);