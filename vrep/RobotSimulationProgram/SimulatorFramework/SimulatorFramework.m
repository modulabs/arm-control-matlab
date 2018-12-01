disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    vrep.simxSynchronous(clientID, true);
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    
    jointHandles=[-1,-1,-1,-1,-1,-1,-1];
    for i=1:7
        [error, jointHandles(i)]=vrep.simxGetObjectHandle(clientID, sprintf('%s%d','LBR4p_joint',i), vrep.simx_opmode_blocking );
    end
    pause(0.1);
    
    for i = 1:7
        [error, Actual_JointPos(i)]=vrep.simxGetJointPosition(clientID, jointHandles(i), vrep.simx_opmode_streaming);
        pause(0.1);    
    end
    
    while 1
        vrep.simxPauseCommunication(clientID, 1);
        
        SimulationTime=vrep.simxGetLastCmdTime(clientID)*SamplingTime;
        
        for i = 1:7
            [error, Actual_JointPos(i)]=vrep.simxGetJointPosition(clientID, jointHandles(i), vrep.simx_opmode_buffer);
        end        
       
        MainControllerLoop;
        DataSave;
        
        for i = 1:7
            
            if(Brake(i)==1)
                vrep.simxSetJointTargetPosition(clientID, jointHandles(i), Init_JointPos(i), vrep.simx_opmode_oneshot);
            else
                vrep.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(Desired_Torque(i))*10e10, vrep.simx_opmode_oneshot);
                vrep.simxSetJointForce(clientID, jointHandles(i), abs(Desired_Torque(i)), vrep.simx_opmode_oneshot);
            end
            
        end
        
        vrep.simxPauseCommunication(clientID, 0);
        
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);
    end
    
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end

vrep.delete();
disp('Program ended');