function robot = model_from_rtb(robot)

n = robot.rtb.n;
robot.B = eye(n);
robot.G = eye(n);
for i=1:n
    robot.B(i,i) = robot.rtb.links(i).Jm;
    robot.G(i,i) = robot.rtb.links(i).G;

    if strcmp(robot.model, 'flexible')
        robot.rtb.links(i).Jm = 0;               % erase
    end
%     robot.rtb.links(i).B = 0;
%     robot.rtb.links(i).Tc = [0 0];
end
robot.K = zeros(n);
robot.D = zeros(n);