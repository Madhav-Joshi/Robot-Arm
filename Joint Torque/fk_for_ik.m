function [T,A] = fk_for_ik(q,Td)
    % This function takes in input as the joint angles and gives a (4,4)
    % transformation matrix from frame zero to frame 6
    l11 = 0.5; l12 = 140.25/1000; 
    l2 = (237.25+50)/1000; 
    l3 = (66.25)/1000;  
    l41 = (237.25)/1000; l42 = 82/1000;
    l5 = 42.25/1000;
    l61 = 99/1000; l62 = 14.35/1000;

    dh = [  pi/2        l11+q(1)    l12     0;
            -pi/2+q(2)  0           l2      0;
            pi/2+q(3)   -l3         0       -pi/2;
            q(4)        l41         0       pi/2;
            q(5)        -l42        0       pi/2;
            pi/2+q(6)   l5+l61      l62     0   ];
    
    A = zeros(4,4,6);
    for i=1:6
        A(:,:,i) = transDH(dh(i,:)); % A represent tranformation from link i-1 to link i
    end
    T = eye(4); 
    for i=1:6
        T = T*A(:,:,i);
    end
    T = T-Td;
end