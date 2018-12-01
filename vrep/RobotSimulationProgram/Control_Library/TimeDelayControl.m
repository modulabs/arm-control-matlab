function [ Desired_Torque ] = ...
    TimeDelayControl( Error_JointPos, Error_JointVel, Actual_JointAcc, Desired_JointAcc, Desired_Torque_Old, M_bar )

Kd = [20,20,20,20,20,20,20];
Kp = [100,100,100,100,100,100,100];

U = Desired_JointAcc + Kd.*Error_JointVel + Kp.*Error_JointPos;
TDE = Desired_Torque_Old - M_bar.*Actual_JointAcc;

Desired_Torque = M_bar.*U + TDE;

end

