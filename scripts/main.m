clear;
clc;
close all;

%% modeling
mdl_puma560;
robot.rtb = p560;
% joint model: select between 'rigid' and 'joint'
robot.model = 'rigid';
% robot.model = 'flexible';
% space : select 'joint' and 'task' space
robot.space = 'joint';
% robot.space = 'task';

% model from rtb
n = robot.rtb.n;
robot = model_from_rtb(robot);

%% simulation
% init state
x(1:n,1) = deg2rad([0, 90, -90, 0, 0, 0]');   % q
x(n+1:2*n,1) = zeros(n,1);                    % q_dot
if strcmp(robot.model, 'flexible')
    x(2*n+1:3*n,1) = zeros(n,1);              % th
    x(3*n+1:4*n,1) = zeros(n,1);              % th_dot
end
robot.tau_ext = zeros(n,1);

% time
dt = 0.001;
robot.dt = dt;

robot.target.qi = x(1:n);
robot.target.qf = deg2rad([0, 90, 0, 0, 90, 0]');
robot.target.ti = 0;
robot.target.tf = robot.target.ti + 5;

% set trajectory and control function
robot.traj = @traj_min_jerk;
robot.control = @control_rigid_passivity;
% robot.control = @control_computed_torque;

tic()
[T, X] = ode45(@(t,x) fdyn(robot, t, x), [robot.target.ti, robot.target.tf], x);
toc()

% animation
figure(1)
for i=1:round(length(T)/100):length(T)
    q = X(i,1:n);
    robot.rtb.plot(q)   
    drawnow
end

% data plot
figure(2)
% re-calculate trajectory to plot
X_des = [];
for i=1:length(T)
    X_des = [X_des; robot.traj(robot, T(i))'];
end

for i=1:n
    subplot(n, 1, i)
    plot(T', rad2deg(X_des(:,i)), 'b')
    hold on
    plot(T', rad2deg(X(:,i)), 'r')
end