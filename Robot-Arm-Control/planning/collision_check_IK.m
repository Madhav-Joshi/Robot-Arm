function Q = collision_check_IK(q)
    Q = zeros(6,4);
    count = 0;
    for i=1:4
        if(~self_collision_check(q(:,i)))
            count = count + 1;
            Q(:,count) = q(:,i);
        end
    end
    if(count>=0)
        Q = Q(:,1:count);
    else
        error("No collision free path");
    end
end