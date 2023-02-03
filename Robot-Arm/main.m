q = [0;0;0;0;0;0];
q_dot = [0;0;0;0;0;0];
q_dotdot = zeros(6,1);

l11 = 648.9/1000; l12 = 196.12/1000; 
l21 = 82/1000; l22 = 315.5/1000; 
l3 = 81.25/1000;  
l4 = 259/1000;
l5 = 42.25/1000;
l61 = 99/1000; l62 = 14.35/1000;

% theta d a alpha revolute; If it is revolute then 1, else prismatic 0
DH = @(q)[  pi/2        l11+q(1)    l12     0       0;
            -pi/2+q(2)  -l21        l22     0       1;
            pi/2+q(3)   -l3         0       -pi/2   1;
            q(4)        l4          0       pi/2    1;
            q(5)        0           0       -pi/2    1;
            -pi/2+q(6)  l5+l61      l62     0       1   ]; % DH Parameters
save('robot_description.mat','DH','l11','l12','l21','l22','l3','l4','l5','l61','l62')

torque6dof(q,q_dot,q_dotdot)