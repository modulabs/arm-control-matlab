%% 데이터 저장

Count = Count + 1;

d_Time(Count)=SimulationTime;
d_ActJPos(Count,1:7)=Actual_JointPos;
d_ActJVel(Count,1:7)=Actual_JointVel;
d_ActJAcc(Count,1:7)=Actual_JointAcc;
d_DesJPos(Count,1:7)=Desired_JointPos;
d_DesJVel(Count,1:7)=Desired_JointVel;
d_DesJAcc(Count,1:7)=Desired_JointAcc;
d_DesTrq(Count,1:7)=Desired_Torque;
d_ErrorJPos(Count,1:7)=Error_JointPos;
d_ErrorJVel(Count,1:7)=Error_JointVel;