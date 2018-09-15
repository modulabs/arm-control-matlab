function tau = tau_ext(robot, t, x, x_env)

n = robot.rtb.n;
q = x(1:n,1);

tau = robot.rtb.jacob0(q)'*[5*sin(4*t);0;0;0;0;0];  % disturbance