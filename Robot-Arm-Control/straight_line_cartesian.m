% Cartesian straight line trajectory energy calculation

% [0.75,pi/2,-pi/2,0,-pi/2,0]
Td = [0.0000    1.0000         0   -0.2590;
   -0.0000    0.0000    1.0000    0.6747;
    1.0000   -0.0000    0.0000    1.2500;
         0         0         0    1.0000];

qi = zeros(6, 1);

[q, q_dot, q_ddot, time_sequence] = controller(Td, qi);
success = visualize_trajectory(q);
%% Generate discrete points (transformation matrices (4,4,n)) 

%% Calculate inverse kinematics for each discrete point
% Get acceptable inverse kinematics of frst point

% Get acceptable inverse kinematics of second point which is closest to
% first point (in joint space), if it is collision free then proceed to
% third point and so on until we reach first point. If the closest IK of 
% sencond point is not collision free then. 
