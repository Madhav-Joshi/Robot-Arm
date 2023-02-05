%% define xf as transformation matrix   
% Td = forwardKinematicsAllJoints([0.75; pi/2; -pi/2; pi/4; -pi/4; 0])
Td = [ 
    0.0000    0.7071   -0.7071   -0.4089;
    0.7071    0.5000    0.5000    0.7142;
    0.7071   -0.5000   -0.5000    1.1752;
         0         0         0    1.0000];


qi = zeros(6, 1);

[q, q_dot, q_ddot, time_sequence, speed, tau, v] = controller(Td, qi);
% success = visualize_trajectory(q);

% for i=1:100
% Calculate screw axis and then interpolate theta for orientation.
% 

