clear;
clc;
close all;

%% modeling
mdl_puma560;
robot.rtb = p560;
robot.model = 'rigid';
% robot.model = 'flexible';
robot.space = 'joint';
% robot.space = 'task';

% model from rtb
n = robot.rtb.n;
robot = model_from_rtb(robot);

%% simulation
% init state
state.q = deg2rad([0, 90, -90, 0, 0, 0]');
state.q_dot = zeros(n,1);
if strcmp(robot.model, 'flexible')
    state.theta = zeros(n,1);
    state.theta_dot = zeros(n,1);
end
tau_ext = zeros(n,1);

% time
t = 0;
dt = 0.001;
robot.dt = dt;
t_total = 10;

target.q = deg2rad([0, 90, 0, 0, 90, 0]');
target.q_dot = zeros(n,1);
target.t = t + 0.1;

T = [];
Q = [];

% trajectory
[Q_des, QD_des, QDD_des]= mtraj(@tpoly, state.q', target.q', (target.t - t)/dt);
i = 1;
while i <= size(Q_des,1)
    state_des.q = Q_des(i,:)';
    state_des.q_dot = QD_des(i,:)';
    state_des.q_ddot = QDD_des(i,:)';
    
    % control
%     tau_m = zeros(n,1); % zero torque
    tau_m = control_rigid_passivity(robot, state, state_des, tau_ext);
    
    % forward dynamics
    state = fdyn(robot, state, tau_m, tau_ext);
    
    T = [T t];
    Q = [Q; state.q'];
        
    i = i+1;
    t = t + dt;
end

% animation
figure(1)
for i=1:length(T)
    q = Q(i,:);
    robot.rtb.plot(q)   
end

% [to do] data plot
figure(2)
for i=1:n
    subplot(n, 1, i)
    plot(T', rad2deg(Q_des(:,i)), 'b')
    hold on
    plot(T', rad2deg(Q(:,i)), 'r')
end