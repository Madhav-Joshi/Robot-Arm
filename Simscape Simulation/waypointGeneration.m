clear; clc;
close all;

%% Adding paths to dependencies and importing required data
addpath(genpath('../Robot-Arm-Control'));
run('main.m');

%% Define xf as transformation matrix   

% Transformation matrix of the final position
Td = [0 1 0 0.225;
      1 0 0 0.8349;
      0 0 1 0.9;
      0 0 0 1 ];

% Initial position of the arm
qi = [0 0 -pi/4 0 0 0]';

%% Finding the optimal trajectory of the arm from initial to final position
[q, q_dot, q_ddot, time_sequence] = controller(Td, qi);

% Forward Trajectory
q_forward = q;
q_dot_forward = q_dot;
q_ddot_forward = q_ddot;

% Backward Trajectory
q_backward = flip(q,2);
q_dot_backward = -flip(q_dot,2);
q_ddot_backward = -flip(q_ddot,2);