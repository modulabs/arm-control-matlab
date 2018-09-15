%%
video = true;
if video
  video_writer = VideoWriter('disturb.mp4', 'MPEG-4');
  open(video_writer);
end
fig_name = 'disturb';

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
x(1:n,1) = deg2rad([60, 150, 60, 60, 150, 60]');   % q
x(n+1:2*n,1) = zeros(n,1);                    % q_dot
if strcmp(robot.model, 'flexible')
    x(2*n+1:3*n,1) = zeros(n,1);              % th
    x(3*n+1:4*n,1) = zeros(n,1);              % th_dot
end
robot.tau_ext = zeros(n,1);
X_des = [];
TAU = [];

% time
dt = 0.001;

robot.target.qi = x(1:n);
robot.target.qf = deg2rad([-60, 60, -30, -60, 60, -60]');
robot.target.ti = 0;
robot.target.tf = robot.target.ti + 2;

% set trajectory and control function
robot.traj = @traj_min_jerk;
robot.control = @control_rigid_passivity;
% robot.control = @control_computed_torque;
% robot.tau_ext = @tau_ext_zero;
robot.tau_ext = @tau_ext_disturb;

tic()
[T X ] = ode45(@(t,x) fdyn(robot, t, x), [robot.target.ti, robot.target.tf], x);
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

%% data plot
% re-calculate trajectory to plot
X_des = [];
for i=1:length(T)
    X_des = [X_des; robot.traj(robot, T(i))'];
end
% re-calculate torque to plot
TAU = [];
for i=1:length(T)
    TAU = [TAU; robot.control(robot, X_des(i,:)', X(i,:)')'];
end

% q, q_des
figure,
f = []; g = [];
for i=1:n
    f = [f subplot(n, 1, i)];
    plot(T', rad2deg(X_des(:,i)), 'b')
    hold on
    plot(T', rad2deg(X(:,i)), 'r')
    title(sprintf('q des, q - %d axis', i))
    xlabel('t(s)')
    ylabel({'q des, q','(deg or mm)'})
    legend('q des', 'q')
    hold on
    grid on
end
linkaxes(f)
g = [g f];
saveas(gcf, sprintf('%s_qdes_q.fig', fig_name))

% q_dot, q_dot_des
figure,
f=[];
for i=1:n
    f = [f subplot(n, 1, i)];
    plot(T', rad2deg(X_des(:,i+6)), 'b')
    hold on
    plot(T', rad2deg(X(:,i+6)), 'r')
    title(sprintf('qdot des, qdot - %d axis', i))
    xlabel('t(s)')
    ylabel({'qdot des', 'qdot(deg or mm)'})
    legend('qdot des', 'qdot')
    hold on
    grid on
end
linkaxes(f)
g = [g f];
saveas(gcf, sprintf('%s_qdot_qdotdes.fig', fig_name))

% torque
figure,
f = [];
for i=1:n
    f = [f subplot(n, 1, i)];
    plot(T', TAU(:,i), 'b')
    hold on
    title(sprintf('tau - %d axis', i))
    xlabel('t(s)')
    ylabel({'tau', '(N.m)'})
    hold on
    grid on
end
linkaxes(f)
g = [g f];
saveas(gcf, sprintf('%s_tau.fig', fig_name))

% q error
figure,
f=[];
for i=1:n
    f = [f subplot(n, 1, i)];
    plot(T', rad2deg(X(:,i) - X_des(:,i)), 'b')
    title(sprintf('q error - %d axis', i))
    xlabel('t(s)')
    ylabel({'q error', '(deg or mm)'})
    hold on
    grid on
end
linkaxes(f)
g = [g f];
saveas(gcf, sprintf('%s_qerror.fig', fig_name))

% q dot error
figure,
f=[];
for i=1:n
    f = [f subplot(n, 1, i)];
    plot(T', rad2deg(X(:,i+6) - X_des(:,i+6)), 'b')
    title(sprintf('qdot error - %d axis', i))
    xlabel('t(s)')
    ylabel({'q dot error', '(deg/s or mm/s)'})
    hold on
    grid on
end
linkaxes(f)
g = [g f];
linkaxes(g, 'x')
saveas(gcf, sprintf('%s_qdoterror.fig', fig_name))

