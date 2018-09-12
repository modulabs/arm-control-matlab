# arm-control-matlab
## Features
- rigid/flexible joint robot dynamics
- trajectory, control algorithm testbed

## Dependency
- Peter Corke's Robotics Toolbox

## Run
1. Edit or Create your own test_*.m
 - Select model between 'rigid' and 'flexible'
 ```matlab
 % joint model: select between 'rigid' and 'joint'
 robot.model = 'rigid';
 % robot.model = 'flexible';
 ```
 - Select space between 'joint' and 'task'
 ```matlab
 % space : select 'joint' and 'task' space
 robot.space = 'joint';
 % robot.space = 'task';
 ```

2. Write your customized trajectory or control function
 - Write trajectory or control function files. Input, output format should be same with given function.
 - Set your functions to robot object in main.m
 ```matlab
 robot.traj = @traj_min_jerk;
 robot.control = @control_rigid_passivity;
 ```

3. Run test_*.m
