function q = allInverseKinematics(Td)
    % Specific to this particular configuration of robot
    % Returns 4 solutions 6x1 matrix
    global DH l11 l12 l2 l3 l4 l5 l61 l62
    dh = DH(zeros(6,1));
    function q_t = signs(s1,s2) % s1 is sign 1 or -1, s2 is also sign 1 or -1
        
        q_t(1) = Td(3,4)-l11+l3;
        p = Td(1:3,4)-transDH(dh(1,1:4));
        D = (p(1)^2+p(2)^2-l2^2-l4^2)/(2*l2*l4);
        % if D>1, solution does not exist
        q_t(3) = atan2(s1*((1-D^2)^0.5),D);
        q_t(2) = atan2(p(2),p(1)) - atan2(-l4*sin(q_t(3)),l2-l4*cos(q_t(2)));

        % if sin(q(5))!=0;
        q_t(5) = atan2(s2*(Td(1,3)^2+Td(2,3)^2)^0.5,Td(3,3));
        q_t(6) = atan2(Td(3,1)/sin(q_t(5)),Td(3,2)/sin(q_t(5)));
        q_t(4) = atan2(-Td(2,3)/sin(q_t(5)),-Td(1,3)/sin(q_t(5)));
        
    end
    q = zeros(6,4);
    q(:,1) = signs(1,1);
    q(:,2) = signs(1,-1);
    q(:,3) = signs(-1,1);
    q(:,4) = signs(-1,-1);
end