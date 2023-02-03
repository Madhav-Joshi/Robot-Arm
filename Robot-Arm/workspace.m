function [reachable_poses] = workspace(joint_limits,task_space)
	sampling_resolution=15;
	reachable_poses=zeros(6,0);
	s=linspace(0,1,sampling_resolution);
    for q2=(joint_limits(2,1)+s*(joint_limits(2,2)-joint_limits(2,1)))
        for q3=(joint_limits(3,1)+s*(joint_limits(3,2)-joint_limits(3,1)))
            for q4=(joint_limits(4,1)+s*(joint_limits(4,2)-joint_limits(4,1)))
                for q5=(joint_limits(5,1)+s*(joint_limits(5,2)-joint_limits(5,1)))
                    for q6=(joint_limits(6,1)+s*(joint_limits(6,2)-joint_limits(6,1)))
                        if~(self_collision_check([0;q2;q3;q4;q5;q6]))
                            [T,~]=fk_for_ik([0;q2;q3;q4;q5;q6],zeros(4,4));
                            reachable_poses=[reachable_poses,[T(1:3,end);T(1:3,3)]];
                        end
                    end
                end
            end
        end
    end
end