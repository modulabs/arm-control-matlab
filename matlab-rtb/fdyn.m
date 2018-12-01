function [x_dot, robot] = fdyn(robot, t, x, tau, tau_ext)

n = robot.rtb.n;



% if strcmp(robot.model, 'rigid')
%     for i=1:n
%         robot.rtb.links(i).Jm = robot.B(i,i);
%     end
if strcmp(robot.model, 'flexible')
    B = robot.B;
    G = robot.G;
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

% % trajectory
% x_des = robot.traj(robot,t);

% actuator torque
if strcmp(robot.model, 'rigid')
    tau_a = tau; %robot.control(robot, x_des, x);
elseif strcmp(robot.model, 'flexible')
    tau_a = K*(theta - q) + D*(theta_dot - q_dot);
end

% % external torque
% tau_ext = robot.tau_ext(robot, t, x, zeros(2*n,1));

% link dynamics
q_ddot = accel(robot.rtb, q', q_dot', tau_a' + tau_ext' );

% motor dynamics
if strcmp(robot.model, 'flexible')
    tau_m = robot.control(robot, x_des, x);
    TAU = [TAU; tau_m'];
    theta_ddot = inv(G*G*B)*(-tau_a + tau_m);
end

% output
x_dot(1:n,1) = q_dot;
x_dot(n+1:2*n,1) = q_ddot;
if strcmp(robot.model, 'flexible')
    x_dot(2*n+1: 3*n,1) = theta_dot;
    x_dot(3*n+1: 4*n,1) = theta_ddot;
end