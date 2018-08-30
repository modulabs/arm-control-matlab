function [state] = fdyn(robot, state, tau_m, tau_ext)

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
q = state.q;
q_dot = state.q_dot;
if strcmp(robot.model, 'flexible')
    theta = state.theta;
    theta_dot = state.theta_dot;
end

% actuator torque
if strcmp(robot.model, 'rigid')
    tau_a = tau_m;
elseif strcmp(robot.model, 'flexible')
    tau_a = K*(theta - q) + D*(theta_dot - q_dot);
end

% link dynamics
q_ddot = accel(robot.rtb, q', q_dot', tau_a' + tau_ext' );
q_dot = q_dot + q_ddot*dt;
q = q + q_dot*dt;

% motor dynamics
if strcmp(robot.model, 'flexible')
    theta_ddot = inv(G*G*B)*(-tau_a + tau_m);
    theta_dot = theta_dot + theta_dot*dt;
    theta = theta + theta_dot*dt;
end

% output
state.q_dot = q_dot;
state.q = q;
if strcmp(robot.model, 'flexible')
    state.theta_dot = theta_dot;
    state.theta = theta;
end