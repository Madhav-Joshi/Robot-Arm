function torque = torque6dof(q,q_dot,q_dotdot)
    % returns how much torque or force each joint should apply to achieve
    % required q, q_dot and q_dotdot
    %% Define 
    n = 6;
    global DH l11 l12 l2 l3 l41 l42 l5 l61 l62;
    dh = DH(q);
    
    % Center of mass
    lc11 = l11/2; lc12 = l12/2;
    lc2 = l2/2;
    lc3 = l3/2;
    lc41 = l41/2; lc42 = l42/2;
    lc5 = l5/2;
    lc61 = l61/2; lc62 = l62/2;

    lc = zeros(3,n+1); % COM coordinates in zero position wrt frame {0}
    lc(:,1) = [0;lc12;lc11]; % COM1 coord in frame 0
    lc(:,2) = [lc2;l12;l11]; % COM2 coord in frame 1
    lc(:,3) = [l2;l12;l11-lc3];
    lc(:,4) = [l2-lc41;l12;l11-l3-lc42];
    lc(:,5) = [l2-l41+lc5;l12;l11-l3-l42];
    lc(:,6) = [l2-l41+l5+lc61;l12;l11-l3-l42+lc62];
    lc = [lc;ones(1,n+1)]; % For multiplication with transformation matrices
    
    T0 = zeros(4,4,n); % Transformation matix from frame 0 to frame i
    for i=1:n
        T = eye(4);
        for j=1:i
            T = T*transDH(dh(j,1:4));
        end
        T0(:,:,i) = T; % Store transformation matrices from 0 to i
    end
    
    m = zeros(n); % mass of links
    for i=1:n
        m(i) = 1;
    end
    
    I0 = zeros(3,3,n); % Inertia of links on COM in ground frame orientation
    for i=1:n
        I0(:,:,i) = eye(3);
    end

%     r = 2.5; 
%     I{1} = zeros(3,3);
%     I{2} = [m{2}*l2^2/12+m{2}*r^2/4 0 0; 0 m{2}*r^2/2 0; 0 0 m{2}*l2^2/12+m{2}*r^2/4];
%     I{3} = [m{3}*l3^2/12+m{3}*r^2/4 0 0; 0 m{3}*l3^2/12+m{3}*r^2/4 0; 0 0 m{3}*r^2/2];
%     I{4} = [m{4}*l4^2/12+m{4}*r^2/4 0 0; 0 m{4}*l4^2/12+m{4}*r^2/4 0; 0 0 m{4}*r^2/2];
%     I{5} = [m{5}*l5^2/12+m{5}*r^2/4 0 0; 0 m{5}*l5^2/12+m{5}*r^2/4 0; 0 0 m{5}*r^2/2];
%     I{6} = [m{6}*l6^2/12+m{6}*r^2/4 0 0; 0 m{6}*l6^2/12+m{6}*r^2/4 0; 0 0 m{6}*r^2/2];

    V0 = [0;0;0;0;0;0]; % Twist of the frame 0
    Vdot0 = [0;0;0;0;0;-9.81];
    
    Ftip = [0;0;0;0;0;0];
    F = zeros(6,7);
    F(:,7) = Ftip; % Defining F_{n+1} = F7 = Ftip

    %% Don't do anything below
     % xc is the COMi coordinates wrt dh frame i-1 
    xc = zeros(4,n+1);
    xc(:,1) = lc(:,1);
    for i=2:n
        xc(:,i)=transformationInverse(T0(:,:,i-1))*lc(:,i);
    end

    I = zeros(3,3,n); % Convert to ith frame orientation
    for i=1:n
        I(:,:,i) = T0(1:3,1:3,i)'*I0(:,:,i)*T0(1:3,1:3,i);
    end

    % Screw axis of joint i in frame i
    A = zeros(6,n);
%     A(:,1) = [0;0;0;0;0;1];
%     A(:,2) = [0;0;1;-lc2;0;0];
%     A(:,3) = [0;0;1;0;0;0];
%     A(:,4) = [0;0;1;lc42;0;0];
%     A(:,5) = [0;0;1;-lc5;0;0];
%     A(:,6) = [0;0;1;lc62;0;0];
    
    for i=1:n
        A(:,i) = [0;0;dh(i,5);(-dh(i,5)*box3([0;0;1])*xc(1:3,i) + (1-dh(i,5))*[0;0;1])];
    end
    
    % M(:,:,i) is the initial transformation matrix of frame i-1 wrt frame i
    M = zeros(4,4,n+1);
    M(:,:,1) = [eye(3) -xc(1:3,1);0 0 0 1];
    for i=2:n+1
        M(:,:,i) = transformationInverse([eye(3) xc(1:3,i);0 0 0 1])*transformationInverse(transDH(dh(i-1,1:4)))*[eye(3) xc(1:3,i-1);0 0 0 1];
    end
    
    % Define T7 which creates 1:6 entries blank
    T = zeros(4,4,n+1);
    T(:,:,n+1) = M(:,:,n+1); 
    V = zeros(6,n);
    Vdot = zeros(6,n);
    
    G = zeros(6,6,n);
    tau = zeros(6);

    %% Forward Iteration: from 1 to n
    % T{i} is transformation matrix of frame i-1 wrt frame i
    for i=1:n
        T(:,:,i) = exp_matrix4(-box6(A(:,i)),q(i))*M(:,:,i);
        % display(T(:,:,i))
        if (i==1)
            V(:,i) = Ad(T(:,:,i))*V0 + A(:,i)*q_dot(i);
            Vdot(:,i) = Ad(T(:,:,i))*Vdot0 + ad_twist(V(:,i))*A(:,i)*q_dot(i) + A(:,i)*q_dotdot(i);
        else
            V(:,i) = Ad(T(:,:,i))*V(:,i-1) + A(:,i)*q_dot(i);
            Vdot(:,i) = Ad(T(:,:,i))*Vdot(:,i-1) + ad_twist(V(:,i))*A(:,i)*q_dot(i) + A(:,i)*q_dotdot(i);
        end
    end

    %% Backward iteration: from n to 1
    for i=6:-1:1
        G(:,:,i) = [I(:,:,i) zeros(3,3); zeros(3,3) m(i)*eye(3)];
        F(:,i) = Ad(T(:,:,i+1))'*F(:,i+1) + G(:,:,i)*Vdot(:,i) - ad_twist(V(:,i))'*G(:,:,i)*V(:,i);
        tau(i) = F(:,i)'*A(:,i); % how much torque is experienced by joint i
    end
    
    torque = zeros(6,1);
    for i=1:6
        torque(i,1) = -tau(i);
    end
end