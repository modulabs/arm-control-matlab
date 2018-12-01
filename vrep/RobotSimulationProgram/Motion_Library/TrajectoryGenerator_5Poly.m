function [ Finish, Desired_Pos, Desired_Vel, Desired_Acc] = ...
    TrajectoryGenerator_5Poly( StartPos, EndPos, Duration, Time )

if(Time > Duration)
    
    Finish = 1;
    Desired_Pos = EndPos;
    Desired_Vel = [0,0,0,0,0,0,0];
    Desired_Acc = [0,0,0,0,0,0,0];
    
else
    
    Finish = 0;
    A0 = StartPos;
    A3 = (20.0*EndPos - 20.0*StartPos) / (2.0*Duration*Duration*Duration);
    A4 = (30.0*StartPos - 30*EndPos) / (2.0*Duration*Duration*Duration*Duration);
    A5 = (12.0*EndPos - 12.0*StartPos) / (2.0*Duration*Duration*Duration*Duration*Duration);
    
    Desired_Pos = A0 + A3*Time^3 + A4*Time^4 + A5*Time^5;
    Desired_Vel = 3.0*A3*Time^2 + 4.0*A4*Time^3 + 5.0*A5*Time^4;
    Desired_Acc = 6.0 * A3*Time + 12.0*A4*Time^2 + 20.0*A5*Time^3;
    
end

end

