function x = traj_min_jerk(robot, t)

n = robot.rtb.n;
qi = robot.target.qi;
qf = robot.target.qf;
duration = robot.target.tf - robot.target.ti;

a0 = qi;
a3 = (20.0*qf - 20.0*qi) / (2.0*duration*duration*duration);
a4 = (30.0*qi - 30*qf) / (2.0*duration*duration*duration*duration);
a5 = (12.0*qf - 12.0*qi) / (2.0*duration*duration*duration*duration*duration);

x(1:n,1) = a0 + a3 * t^3 + a4 * t^4 + a5 * t^5;
x(n+1:2*n,1) = 3.0 * a3 * t^2 + 4.0 * a4 * t^3 + 5.0 * a5 * t^4;
x(2*n+1:3*n,1) = 6.0 * a3 * t + 12.0 * a4 * t^2 + 20.0 * a5 * t^3;