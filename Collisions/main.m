clc
clear
close all
%% defining the DH params of robot
global dh  l11 l12 l2 l3 l41 l42 l5 l61 l62
l11 = 500/1000;
l12 = 218/1000;
l2 = (237.25+50)/1000;
l3 = (66.25)/1000;
l41 = (237.25)/1000;
l42 = 82/1000;
l5 = 42.25/1000;
l61 = 99/1000;
l62 = 14.35/1000;
dh = @(q)[ pi/2        l11+q(1)    l12     0;
           -pi/2+q(2)  0           l2      0;
           pi/2+q(3)   -l3         0       -pi/2;
           q(4)        l41         0       pi/2;
           q(5)        -l42        0       pi/2;
           pi/2+q(6)   l5+l61      l62     0   ];
%% defining the bounding boxes
n_links=6;
f_c=zeros(3,n_links);
b_dim=zeros(3,n_links);
%link1
f_c(:,1)=[-218;-720*0.5+90;100]/1000;
b_dim(:,1)=[200;720;1200]/1000;

%link2
f_c(:,2)=[-349.5*0.5+17.25;0;0]/1000;
b_dim(:,2)=[349.5;50;50]/1000;

%link3
f_c(:,3)=[0;-59.5*0.5+17.25+4;0]/1000;
b_dim(:,3)=[49;59.5;42]/1000;

%link4
f_c(:,4)=[0;-230*0.5+16.75;0]/1000;
b_dim(:,4)=[50;230;50]/1000;

%link5
f_c(:,5)=[0;0;59.5*0.5-17.25]/1000;
b_dim(:,5)=[49;42;59.5]/1000;

%link6
f_c(:,6)=[-23.35;0;-97*0.5]/1000;
b_dim(:,6)=[109;84;97]/1000;
%% creating collision boxes
clc
close all;
collision_boxes{n_links}=[];
[T,A]=fk_for_ik([0;pi/2;pi/3;pi/6;pi/4;pi/3],zeros(4,4));
Link_Frame=eye(4);
for i=1:n_links
	collision_boxes{i}=collisionBox(b_dim(1,i),b_dim(2,i),b_dim(3,i));
    Link_Frame=Link_Frame*A(:,:,i);
    collision_boxes{i}.Pose=Link_Frame*[eye(3),f_c(:,i);0 0 0 1]; 
end
for i=1:n_links
    show(collision_boxes{i})
    hold on
end
%% checking collisions
colliding_state=[];
for q2=-pi/2:0.5:pi/2
    for q3=-pi:0.5:pi
        for q4=-pi:0.5:pi
            for q5=-pi:0.5:pi
                for q6=-pi:0.5:pi
                    [T,A]=fk_for_ik([0;q2;q3;q4;q5;q6],zeros(4,4));
                    Link_Frame=eye(4);
                    for i=1:n_links
                        collision_boxes{i}=collisionBox(b_dim(1,i),b_dim(2,i),b_dim(3,i));
                        Link_Frame=Link_Frame*A(:,:,i);
                        collision_boxes{i}.Pose=Link_Frame*[eye(3),f_c(:,i);0 0 0 1]; 
                    end
                    for i=1:(n_links-1)
                        for j=(i+1):n_links
                            if(checkCollision(collision_boxes{i},collision_boxes{j}))
                                colliding_state=[colliding_state,[0;q2;q3;q4;q5;q6]];
                                disp("colliding state at q2:"+q2+" q3:"+q3+" q4:"+q4+" q5:"+q5+" q6:"+q6)
                            end
                        end
                    end                                
                end
            end
        end
    end
end
