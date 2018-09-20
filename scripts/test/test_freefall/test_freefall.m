%%
video = true;
if video
  video_writer = VideoWriter('freefall.mp4', 'MPEG-4');
  open(video_writer);
end
fig_name = 'freefall';

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
for i=1:n
%     robot.rtb.links(i).Jm = 0;      % actuator inertia 
%     robot.rtb.links(i).G = 1;       % gear ratio
    robot.rtb.links(i).B = 0;       % viscous friction
    robot.rtb.links(i).Tc = [0 0];  % Coulomb friction
end

%% simulation
% init state
x(1:n,1) = deg2rad([-60, 60, -30, -60, 60, -60]');   % q
x(n+1:2*n,1) = zeros(n,1);                    % q_dot
x_des = [x; zeros(n,1)];                      % init desired state including acceleration
tau = zeros(n,1);   tau_ext = zeros(n,1);

% save
T = [];
X = [];     X_des = [];
TAU = [];

% set trajectory and control function
t = 0; tf = 5; dt = 0.001;

tic()
while t <= tf
    % save
    T = [T; t];
    X = [X; x'];
    TAU = [TAU; tau'];
    
    % control
    tau = zeros(n,1);
    
    % external torque
    tau_ext = zeros(n,1);
        
    % forward dynamics
    % using ode45
%     opts = odeset('RelTol',1e-3);
%     [Tode, Xode ] = ode45(@(t,x) fdyn(robot, t, x, tau, tau_ext), [t, t + dt], x, opts);
%     t = Tode(end);
%     x = Xode(end,:)';
    % using simple euler integration
    x_dot = fdyn(robot, t, x, tau, tau_ext);
    x = x + x_dot*dt;
    t = t + dt;
end
toc()

%% animation
h = figure;
for i=1:round(length(T)/100):length(T)
    q = X(i,1:n);
    robot.rtb.plot(q);
    if video
        writeVideo(video_writer, getframe(h));
    end
    drawnow
end
if video
    close(video_writer);
end