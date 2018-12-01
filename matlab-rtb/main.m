clear;
clc;
close all;

%% add path
test_path = 'test/test_coriolis';
addpath('.', 'control', 'trajectory', test_path)

%% run your test file
old_path = cd(test_path);
test_coriolis
cd(old_path)