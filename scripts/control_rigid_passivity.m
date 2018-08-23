function tau = contro_rigid_passivity(robot, x, x_des)

n = robot.rtb.n;
tau_ext = robot.tau_ext;

tau = zeros(n,1);