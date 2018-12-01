pause(0.2);
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
pause(1);
vrep.simxFinish(clientID);
pause(0.2);
vrep.delete();
disp('Program ended');