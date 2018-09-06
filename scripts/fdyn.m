function [x] = fdyn(robot, t, x)

n = robot.rtb.n;
dt = robot.dt;

B = robot.B;
G = robot.G;

if strcmp(robot.model, 'rigid')
    for i=1:n
        robot.rtb.links(i).Jm = robot.B(i,i);
    end
elseif strcmp(robot.model, 'flexible')
    K = robot.K;
    D = robot.D;
end

% state
q = x(1:n);
q_dot = x(n+1:2*n);
if strcmp(robot.model, 'flexible')
    theta = x(2*n+1:3*n);
    theta_dot = x(3*n+1:4*n);
end

% trajectory
x_des = robot.traj(robot,t);



% actuator torque
if strcmp(robot.model, 'rigid')
    tau_a = robot.control(robot, x, x_des);
elseif strcmp(robot.model, 'flexible')
    tau_a = K*(theta - q) + D*(theta_dot - q_dot);
end

% external torque
%robot.tau_ext = robot.rtb.jacob0(q)'*[5*sin(2*t);0;0;0;0;0];  % disturbance
tau_ext = robot.tau_ext;

% link dynamics
q_ddot = accel(robot.rtb, q', q_dot', tau_a' + tau_ext' );

% motor dynamics
if strcmp(robot.model, 'flexible')
    tau_m = robot.control(robot, x, x_des);
    theta_ddot = inv(G*G*B)*(-tau_a + tau_m);
end

% output
x(1:n) = q_dot;
x(n+1:2*n) = q_ddot;
if strcmp(robot.model, 'flexible')
    x(2*n+1: 3*n) = theta_dot;
    x(3*n+1: 4*n) = theta_ddot;
end