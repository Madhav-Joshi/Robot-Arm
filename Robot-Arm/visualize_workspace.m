function [flag]=visualize_workspace(reachable_poses,task_space)
    %this function plots the workspace. On left all the possible points are
    %shown and on right all possible the oreintation of the position which we select on
    %the left plot is shown
    
    delta_x=0.02;
    delta_y=0.02;
    
    % gridlines ---------------------------
    hold on
    g_x=task_space(1,1):delta_x:task_space(1,2); % user defined grid Y [start:spaces:end]
    g_y=task_space(2,1):delta_y:task_space(2,2); % user defined grid X [start:spaces:end]
    for i=1:length(g_x)
       plot([g_x(i) g_x(i)],[g_y(1) g_y(end)],'k:') %y grid lines
       hold on    
    end
    for i=1:length(g_y)
       plot([g_x(1) g_x(end)],[g_y(i) g_y(i)],'k:') %x grid lines
       hold on    
    end
    points_x=(g_x(1:end-1)+g_x(2:end))*0.5;
    points_y=(g_y(1:end-1)+g_y(2:end))*0.5;

end