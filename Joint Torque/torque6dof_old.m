function torque = torque6dof_old(t,t_dot,t_dotdot)
    %% Define 
    n = 6;
    % syms l11 l12 lc11 lc12 l2 lc2 l3 lc3 l4 lc4 l5 lc5 l6 lc6
    l11 = 50; l12 = 50; lc11 = 25; lc12 = 25;
    l2 = 50; lc2 = 25; l3 = 50; lc3 = 25; l4 = 50; lc4 = 25;
    l5 = 0.1; lc5 = 0.05; l6 = 0.1; lc6 = 0.05;
    
    % M{i} is the initial transformation matrix of frame i-1 wrt frame i
    M{1} = [eye(3) [0;-lc12;-lc11]; 0 0 0 1];

    M{2} = [0   1   0   -(l12-lc12) ; 
            -1  0   0   lc2         ; 
            0   0   1   -(l11-lc11) ; 
            0   0   0   1           ];

    M{3} = [0   -1  0   -(l2-lc2)   ; 
            1   0   0   0           ; 
            0   0   1   -lc3        ; 
            0   0   0   1           ];

    M{4} = [0   1   0   0           ; 
            0   0   -1  (l3-lc3)    ; 
            -1  0   0   -lc4        ; 
            0   0   0   1           ];

    M{5} = [1   0   0   0           ; 
            0   0   -1  (l4-lc4)    ; 
            0   1   0   -lc5        ; 
            0   0   0   1           ];

    M{6} = [0   1   0   0           ; 
            0   0   1   -(l5-lc5)   ; 
            1   0   0   -lc6        ; 
            0   0   0   1           ];
    M{7} = [eye(3) [0;0;-(l6-lc6)]; 0 0 0 1];
    
    % Screw axis of joint i in frame i
    A{1} = [0;0;0;0;0;1];
    A{2} = [0;0;1;lc2;0;0];
    A{3} = [0;0;1;0;0;0];
    A{4} = [0;0;1;0;0;0];
    A{5} = [0;0;1;0;0;0];
    A{6} = [0;0;1;0;0;0];
    
    m{n} = []; % mass of links
    I{n} = []; % Moment of inertial of links
    % for i=1:6
    %     I{i} = sym("I"+string(i),[3 3]);
    %     m{i} = sym("m"+string(i));
    % end
    for i=1:4
        m{i} = 1;
    end
    m{5} = 0.2; m{6} = 0.2;
    
    r = 2.5; 
    I{1} = zeros(3,3);
    I{2} = [m{2}*l2^2/12+m{2}*r^2/4 0 0; 0 m{2}*r^2/2 0; 0 0 m{2}*l2^2/12+m{2}*r^2/4];
    I{3} = [m{3}*l3^2/12+m{3}*r^2/4 0 0; 0 m{3}*l3^2/12+m{3}*r^2/4 0; 0 0 m{3}*r^2/2];
    I{4} = [m{4}*l4^2/12+m{4}*r^2/4 0 0; 0 m{4}*l4^2/12+m{4}*r^2/4 0; 0 0 m{4}*r^2/2];
    I{5} = [m{5}*l5^2/12+m{5}*r^2/4 0 0; 0 m{5}*l5^2/12+m{5}*r^2/4 0; 0 0 m{5}*r^2/2];
    I{6} = [m{6}*l6^2/12+m{6}*r^2/4 0 0; 0 m{6}*l6^2/12+m{6}*r^2/4 0; 0 0 m{6}*r^2/2];
    
    V0 = [0;0;0;0;0;0]; % Twist of the frame 0
    Vdot0 = [0;0;0;0;0;-9.81];
    
    Ftip = [0;0;0;0;0;0];
    F{7} = Ftip; % Defining F_{n+1} = F7 = Ftip
    
    % Don't do anything below
    T{n+1} = M{n+1}; % Define T7 which creates 1:6 entries blank
    V{n} = [];
    Vdot{n} = [];
    
    G{n} = [];
    tau{n} = [];

    %% Forward Iteration: from 1 to n
    % T{i} is transformation matrix of frame i-1 wrt frame i
    for i=1:n
        T{i} = exp_matrix4(-box6(A{i}),t(i))*M{i};
        % display(T{i})
        if (i==1)
            V{i} = Ad(T{i})*V0 + A{i}*t_dot(i);
            Vdot{i} = Ad(T{i})*Vdot0 + ad_twist(V{i})*A{i}*t_dot(i) + A{i}*t_dotdot(i);
        else
            V{i} = Ad(T{i})*V{i-1} + A{i}*t_dot(i);
            Vdot{i} = Ad(T{i})*Vdot{i-1} + ad_twist(V{i})*A{i}*t_dot(i) + A{i}*t_dotdot(i);
        end
    end

    %% Backward iteration: from n to 1
    for i=6:-1:1
        G{i} = [I{i} zeros(3,3); zeros(3,3) m{i}*eye(3)];
        F{i} = Ad(T{i+1})'*F{i+1} + G{i}*Vdot{i} - ad_twist(V{i})'*G{i}*V{i};
        tau{i} = F{i}'*A{i}; 
    end
    
    torque = zeros(6,1);
    for i=1:6
        torque(i,1) = -tau{i};
    end
end