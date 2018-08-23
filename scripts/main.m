clear;
clc;

%% modeling
mdl_puma560;
my_robot.rtb = p560;

% model from rtb
n = my_robot.rtb.n;
my_robot.B = eye(n);
my_robot.G = eye(n);
for i=1:n
    my_robot.B(i,i) = my_robot.rtb.links(i).Jm;
    my_robot.G(i,i) = my_robot.rtb.links(i).G;
    
    my_robot.rtb.links(i).Jm = 0;               % erase
    my_robot.rtb.links(i).B = 0;
    my_robot.rtb.links(i).Tc = 0;
end
my_robot.K = zeros(n);
my_robot.D = zeros(n);

% init
my_robot.tau_m = zeros(n,1);
my_robot.tau_ext = zeros(n,1);
my_robot.control = @control_rigid_passivity;
% my_robot.control = @control_flexible_passivity_func;
dynamics = @fdyn_rigid_joint;
% dynamics = @fdyn_flexible_joint;

%% simulation
ts = [0 10];
q0 = zeros(n,1);
q_dot0 = zeros(n,1);
theta0 = zeros(n,1);
theta_dot0 = zeros(n,1);
x0 = [q0; q_dot0]; %; theta0; theta_dot0];

% solve
% [t x] = ode45(@(t,x) fdyn_flexible_joint(t,x,my_robot), ts, x0);
[t x] = ode45(@(t,x) fdyn_rigid_joint(t,x,my_robot), ts, x0);

% plot
figure(1)
for i=1:length(t)
    q = x(i,1:6);
    my_robot.rtb.plot(q)
end