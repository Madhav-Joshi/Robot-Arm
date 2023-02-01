function T = forwardKinematicsAllJoints(q)
    % This function takes in input as the joint angles and gives a (4,4,6)
    % array of joint transformations from frame zero to frame i (i=1:6)
    % syms l11 l12 lc11 lc12 l21 l22 lc21 lc22 l3 lc3 l41 l42 l43 lc41 lc42 l5 lc5 l6 lc6
    l11 = 0.5; l12 = 140.25/1000; 
    l2 = (237.25+50)/1000; 
    l3 = (66.25)/1000;  
    l41 = (237.25)/1000; l42 = 82/1000;
    l5 = 42.25/1000;
    l61 = 99/1000; l62 = 14.35/1000;

    lc11 = l11/2; lc12 = l12/2;
    lc2 = l2/2;
    lc3 = l3/2;
    lc41 = l41/2; lc42 = l42/2;
    lc5 = l5/2;
    lc61 = l61/2; lc62 = l62/2;

    dh = [  pi/2        l11+q(1)    l12     0;
            -pi/2+q(2)  0           l2      0;
            pi/2+q(3)   -l3         0       -pi/2;
            q(4)        l41         0       pi/2;
            q(5)        -l42        0       pi/2;
            pi/2+q(6)   l5+l61      l62     0   ];

    A = zeros(4,4,6);
    T = zeros(4,4,6)+eye(4);
    for i=1:6
        A(:,:,i) = transDH(dh(i,:)); % A represent tranformation from link i-1 to link i
        for j=1:i
            T(:,:,i) = T(:,:,i)*A(:,:,j);
        end
    end
end