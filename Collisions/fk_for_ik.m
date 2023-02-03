function [T,A] = fk_for_ik(q,Td)
    % This function takes in input as the joint angles and gives a (4,4)
    % transformation matrix from frame zero to frame 6
    global dh  l11 l12 l2 l3 l41 l42 l5 l61 l62
    dh_=dh(q);
    A = zeros(4,4,6);
    for i=1:6
        A(:,:,i) = transDH(dh_(i,:)); % A represent tranformation from link i-1 to link i
    end
    T = eye(4); 
    for i=1:6
        T = T*A(:,:,i);
    end
    T = T-Td;
end