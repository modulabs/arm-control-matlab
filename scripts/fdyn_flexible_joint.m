function [x_dot] = fdyn_flexible_joint(t, x, robot)

n = robot.rtb.n;

tau_ext = robot.tau_ext;

B = robot.B;
G = robot.G;
K = robot.K;
D = robot.D;

% state
q = x(1:n);
q_dot = x(n+1:2*n);
theta = x(2*n+1:3*n);
theta_dot = x(3*n+1:4*n);

tau_m = robot.control(q, q_dot, theta, theta_dot);

% flexible joint dynamics
tau_a = K*(theta - q) + D*(theta_dot - q_dot);

% link dynamics
q_ddot = accel(robot.rtb, q', q_dot', tau_a' + tau_ext' );

% motor dynamics
theta_ddot = G*G*inv(B)*(-tau_a + tau_m);

robot.tau_a = tau_a;
x_dot = [q_dot; q_ddot; theta_dot; theta_ddot];