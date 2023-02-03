% hold on
for q1 = 0:0.1:1
    for q2 = 0:pi/20:pi/2
        for q3 = 0:pi/20:pi/2
            for q4 = 0:pi/20:pi/2
                for q5 = 0:pi/20:pi/2
                    for q6 = 0:pi/20:pi/2
                        FK = forwardKinematicsAllJoints([q1;q2;q3;q4;q5;q6]);
                        plot3([0,FK(1,4,1),FK(1,4,2),FK(1,4,3),FK(1,4,4),FK(1,4,5),FK(1,4,6)], ...
                                [0,FK(2,4,1),FK(2,4,2),FK(2,4,3),FK(2,4,4),FK(2,4,5),FK(2,4,6)], ...
                                [0,FK(3,4,1),FK(3,4,2),FK(3,4,3),FK(3,4,4),FK(3,4,5),FK(3,4,6)])
                        xlim([-1 1]); ylim([-1 1]); zlim([0 1]);
                        title('Robot stick figure')
                        grid on
                        pause(0.001)
                    end
                end
            end
        end
    end
end
