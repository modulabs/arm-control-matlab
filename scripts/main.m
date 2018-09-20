clear;
clc;
close all;

%% add path
test_path = 'test/test_rigid_passivity';
addpath('.', 'control', 'trajectory', test_path)

%% run your test file
old_path = cd(test_path);
test_rigid_passivity
cd(old_path)