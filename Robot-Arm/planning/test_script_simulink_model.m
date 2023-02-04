%% initial and final points
% qi = [0;0;0;0;0;0];
% qf = [0.75;pi/2;pi/2;pi/2;pi/2;pi/2];

qf = [0;0;0;0;0;0];
qi = [0.75;pi/2;pi/2;pi/2;pi/2;pi/2];

%% obtain joint value sequences for trajectory with minimal energy
tic
[q_final, q_dot_final, q_ddot_final, time_sequence_final, energy_final] = trajectory_planning(qi,qf);
toc
success = visualize_trajectory(q_final);

% debug
% [energy] = calculate_trajectory_energy(time_sequence_final, q_final, q_dot_final, q_ddot_final);
% waypoints_debug = waypoints_final(:, 401:500) + 0.1
% [q_, q_dot, q_ddot, time_sequence] = plan_velocity_constant_profile(waypoints_debug)
%% defining timeseries objects for joint
q_signal = timeseries(q_final,time_sequence_final); % meters, rad, rad, rad, rad, rad
q_dot_signal = timeseries(q_dot_final,time_sequence_final); % m/s, rad/s, rad/s, rad/s, rad/s, rad/s
q_ddot_signal = timeseries(q_ddot_final,time_sequence_final); % m/s^2, rad/s^2, rad/s^2, rad/s^2, rad/s^2 