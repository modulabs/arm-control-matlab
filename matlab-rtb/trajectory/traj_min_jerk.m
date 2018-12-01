function x = traj_min_jerk(robot, t)

n = robot.rtb.n;
ti = robot.target.ti;
tf = robot.target.tf;
dt = t - ti;
qi = robot.target.qi;
qf = robot.target.qf;
duration = robot.target.tf - robot.target.ti;

a0 = qi;
a3 = (20.0*qf - 20.0*qi) / (2.0*duration*duration*duration);
a4 = (30.0*qi - 30*qf) / (2.0*duration*duration*duration*duration);
a5 = (12.0*qf - 12.0*qi) / (2.0*duration*duration*duration*duration*duration);

if t < ti
    x(1:n,1) = qi;
    x(n+1:2*n,1) = zeros(n,1);
    x(2*n+1:3*n,1) = zeros(n,1);
elseif  t > tf
    x(1:n,1) = qf;
    x(n+1:2*n,1) = zeros(n,1);
    x(2*n+1:3*n,1) = zeros(n,1);
else
    x(1:n,1) = a0 + a3 * dt^3 + a4 * dt^4 + a5 * dt^5;
    x(n+1:2*n,1) = 3.0 * a3 * dt^2 + 4.0 * a4 * dt^3 + 5.0 * a5 * dt^4;
    x(2*n+1:3*n,1) = 6.0 * a3 * dt + 12.0 * a4 * dt^2 + 20.0 * a5 * dt^3;
end