close all; clear; clc;

addpath('SimulatorFramework');
addpath('Edit_Controller');
addpath('SimulatorFramework\Vrep_Library');
addpath('RoboticsToolbox_Library\rvctools');
addpath('Control_Library');
addpath('Motion_Library');

startup_rvc;
InitParameter;
SimulatorFramework;