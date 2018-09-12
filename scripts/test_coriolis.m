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

q = [1;1;1;1;1;1];
qd = [1;1;1;1;1;1];
qda = [2;2;2;2;2;2];
qdd = [1;1;1;1;1;1];

% inverse dynamics
M = robot.rtb.inertia(q');

% compare C matrix
C = robot.rtb.coriolis(q', qd')
C_ = zeros(robot.rtb.n);
g = rne_dh_(robot.rtb, q', zeros(1,n), zeros(1,n), zeros(1,n));
for i=1:robot.rtb.n
    unit_qd = zeros(robot.rtb.n,1);
    unit_qd(i) = 1;
    C_(:,i) = rne_dh_(robot.rtb, q', qd', unit_qd', zeros(1,n)) - g; 
end
C_

% compare C(q,qd)*qd 
Cq = (C*qd)'
Cq_ = rne_dh(robot.rtb, q', qd', zeros(1,n)) - rne_dh(robot.rtb, q', zeros(1,n), zeros(1,n))
Cq__ = rne_dh_(robot.rtb, q', qd', qd', zeros(1,n) ) ...
    - rne_dh_(robot.rtb, q', zeros(1,n), zeros(1,n), zeros(1,n))

% compare C(q,qd)*qda
Cqa = (C*qda)'
Cqa__ = rne_dh_(robot.rtb, q', qd', qda', zeros(1,n) ) - g

