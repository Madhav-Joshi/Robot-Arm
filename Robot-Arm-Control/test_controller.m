%% define xf as transformation matrix   
Td = [0.0000 0.7071   -0.7071   -0.3589;
    1.0000   -0.0000    0.0000    0.5478;
    0.0000   -0.7071   -0.7071    0.5858;
         0         0         0    1.0000];

% Td=[1.0000    0.0000    0.0000    0.2734;
%    -0.0000    1.0000         0    0.5334;
%          0         0    1.0000    1.3769;
%          0         0         0    1.0000];

qi = zeros(6, 1);

[q, q_dot, q_ddot, time_sequence] = controller(Td, qi);
success = visualize_trajectory(q);
