function [x_dot] = fdyn_rigid_joint(t, x, robot)

n = robot.rtb.n;

tau_ext = robot.tau_ext;

B = robot.B;
G = robot.G;

for i=1:n
    robot.rtb.links(i).Jm = robot.B(i,i);
end

% state
q = x(1:n);
q_dot = x(n+1:2*n);
% theta = q;
% theta_dot = q_dot;

tau_m = robot.control(robot, q, q_dot);

% rigid joint dynamics
tau_a = tau_m;

% link dynamics
q_ddot = accel(robot.rtb, q', q_dot', tau_a' + tau_ext' );

% % motor dynamics
% theta_ddot = inv(G*G*B)*(-tau_a + tau_m);

% robot.tau_a = tau_a;
x_dot = [q_dot; q_ddot];