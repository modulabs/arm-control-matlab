function x = traj_hold(robot, t)

n = robot.rtb.n;

q = robot.target.qf;
qd = zeros(n,1);
qdd = zeros(n,1);

x(1:n) = q;
x(n+1:2*n) = qd;
x(2*n+1,3*n) = qdd;