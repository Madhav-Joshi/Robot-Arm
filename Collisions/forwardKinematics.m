function T = forwardKinematics(q)
    % This function takes in input as the joint angles and gives a (4,4,6)
    % array of joint transformations from frame zero to frame i (i=1:6)
    % syms l11 l12 lc11 lc12 l21 l22 lc21 lc22 l3 lc3 l41 l42 l43 lc41 lc42 l5 lc5 l6 lc6
    l11 = 0.50; l12 = 0.50; lc11 = 0.25; lc12 = 0.25;
    l21 = 0.50; l22 = 0.50; lc21 = 0.25; lc22 = 0.25; l3 = 0.50; lc3 = 0.25; 
    l41 = 0.25; l42 = 0.50; l43 = 0.25; lc41 = 0.25; lc42 = 0.25;
    l5 = 0.1; lc5 = 0.05; l6 = 0.1; lc6 = 0.05;

    dh = [  pi/2        l11+q(1)    l12     0;
            -pi/2+q(2)  l22       l21      0;
            pi/2+q(3)   0      0       -pi/2;
            q(4)        l3+l41+l43      0       pi/2;
            -pi/2+q(5)   -l42      0       -pi/2;
            q(6)        l5+l6      0       0   ];
    A = zeros(4,4,6);
    T = zeros(4,4,6)+eye(4);
    for i=1:6
        A(:,:,i) = transDH(dh(i,:));
        for j=1:i
            T(:,:,i) = T(:,:,i)*A(:,:,j);
        end
    end
end