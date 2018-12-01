%% 변수 선언

Init_JointPos=deg2rad([0,0,0,0,0,0,0]);
Brake=[0,0,0,0,0,0,0];

M_bar=[0.02,0.04,0.004,0.05,0.001,0.001,0.0001];
SamplingTime=0.001;

Actual_JointPos=[0,0,0,0,0,0,0];
Actual_JointPos_Old=[0,0,0,0,0,0,0];
Actual_JointVel=[0,0,0,0,0,0,0];
Actual_JointVel_Old=[0,0,0,0,0,0,0];
Actual_JointAcc=[0,0,0,0,0,0,0];
Desired_Torque=[0,0,0,0,0,0,0];
Desired_Torque_Old=[0,0,0,0,0,0,0];
Error_JointPos=[0,0,0,0,0,0,0];
Error_JointVel=[0,0,0,0,0,0,0];

StartJointPos(1,1:7)=deg2rad([0,0,0,0,0,0,0]);
EndJointPos(1,1:7)=deg2rad([70,70,70,70,70,70,70]);
StartJointPos(2,1:7)=deg2rad([70,70,70,70,70,70,70]);
EndJointPos(2,1:7)=deg2rad([0,0,0,0,0,0,0]);
StartJointPos(3,1:7)=deg2rad([0,0,0,0,0,0,0]);
EndJointPos(3,1:7)=deg2rad([-70,-70,-70,-70,-70,-70,-70]);
StartJointPos(4,1:7)=deg2rad([-70,-70,-70,-70,-70,-70,-70]);
EndJointPos(4,1:7)=deg2rad([0,0,0,0,0,0,0]);
Finish=0;
Step=1;
TotalStep=4;
Duration=2;
Time=0;
Count=0;