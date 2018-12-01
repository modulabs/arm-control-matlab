%% Controller MainLoop
%% Actual_JointPos -> 현재 조인트 값 저장 변수
%% Desired_Torque -> 토크 지령 값 저장 변수

Time = Time + SamplingTime;

Actual_JointVel = (Actual_JointPos - Actual_JointPos_Old)/SamplingTime;
Actual_JointAcc = (Actual_JointVel - Actual_JointVel_Old)/SamplingTime;
Actual_JointPos_Old = Actual_JointPos;
Actual_JointVel_Old = Actual_JointVel;

if(Finish)
    Step=Step+1;
    Finish=0;
    Time=0;
elseif(TotalStep==(Step-1))

else
    [ Finish, Desired_JointPos, Desired_JointVel, Desired_JointAcc] = ...
        TrajectoryGenerator_5Poly( StartJointPos(Step,1:7), EndJointPos(Step,1:7), Duration, Time );
end

Error_JointPos = Desired_JointPos - Actual_JointPos;
Error_JointVel = Desired_JointVel - Actual_JointVel;

[ Desired_Torque ] = ...
    TimeDelayControl( Error_JointPos, Error_JointVel, Actual_JointAcc, Desired_JointAcc, Desired_Torque_Old, M_bar );

Desired_Torque_Old = Desired_Torque;