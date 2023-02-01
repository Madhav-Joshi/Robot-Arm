%% Define
% Notations followed from the book Modern Robotics by Kevin Lynch and Frank
% Park; note: words robot, bot and body are used interchangeably
% t = sym('t',[3 1]); % t = theta = vector of relative joint angles
t = [0;0;0]; 
% t_dot = sym('t_dot',[3 1]); % t_dot = time dervative of theta
t_dot = [0;0;0];
% t_dotdot = sym('t_dotdot',[3 1]); % t_dotdot = double time derivative of theta
t_dotdot = [0;0;0];
% syms l1 l2 l3 lc1 lc2 lc3;  % li - length of link i and lci = distance of center of mass of link i from joint i
l1=0.05; l2=0.40; l3=0.355; lc1=0.025; lc2=0.2; lc3=0.1775;
M10 = [eye(3) [0;0;-lc1];0 0 0 1]; % transformation matrix from frame 1 to frame 0 when theta=0
M21 = [eye(3) [-lc2;0;-(l1-lc1)];0 0 0 1]; % transformation matrix from frame 2 to frame 1 when theta=0
M32 = [eye(3) [-(lc3+l2-lc2);0;0];0 0 0 1]; % transformation matrix from frame 3 to frame 2 when theta=0
M43 = [eye(3) [-(l3-lc3);0;0];0 0 0 1]; % transformation matrix from frame 4 to frame 3 when theta=0

A1 = [0;0;1;0;0;0]; % Screw axis of joint 1
A2 = [0;-1;0;0;0;lc2]; % Screw axis of joint 2
A3 = [0;-1;0;0;0;lc3]; % Screw axis of joint 3-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% rcb = sym('rcb',[3 1]); % Distance vector from center of mass to base of robot arm
rcb = [0;0;0];
% wbot = sym('wbot',[3 1]);% Angular velocity of robot body wrt body frame at center of mass attached to robot body
wbot = [0;0;0];
% vbot = sym('vbot',[3 1]); % Velocity of robot body wrt body frame again
vbot = [0;0;0];
Vbot = [wbot;vbot]; % Twist wrt body frame again

T0bot = [eye(3) -rcb;0 0 0 1]; % Transformation matrix from frame 0 i.e. base of robot arm to body frame
V0 = Ad(T0bot)*Vbot; % Twist of bot in frame 0

% wbot_dot = sym('wbot_dot',[3 1]); % Angular acceleration in body frame
wbot_dot = [0;0;0];
% vbot_dot = sym('vbot_dot',[3 1]); % Linear acceleration in body frame
vbot_dot = [0;0;-9.81];
Vbot_dot = [wbot_dot;vbot_dot]; % Twist of bot in body frame 
V0_dot = Ad(T0bot)*Vbot_dot; % Twist of bot in frame 0

% mo = sym('mo',[3 1]); % Torque or moment at tip of the robot arm in the frame of the tip
mo = [0;0;0];
% f = sym('f',[3 1]); % Force at the tip of the robot arm in the frame of the tip
% f = [-2.5*9.81*sin(t(2)+t(3));0;-2.5*9.81*cos(t(2)+t(3))];
f = [0;0;0];
Ftip = [mo;f]; % Wrench at the tip of the arm in the frame of the tip
F4 = Ftip; % Defining F_{n+1} = F4 = Ftip

% syms m1 m2 m3 % mi = mass of link i
% approx mass of motor be 150gram and kg/len ~ 0.625
m1_lm = 0.18; m2_lm = 0.4; m3_lm = 0.37;
motor_mass = 0.15;
m1 = 0.03; m2 = 0.25; m3 = 0.22;

% I1 = sym('I1',[3 3]); % Inertia of link 1 in the frame of link 1 at its center of mass 
I1 = [[m1*l1*l1/12+motor_mass*lc1*lc1;0;0],[0;0;0],[0;0;m1*l1*l1/12+motor_mass*lc1*lc1]];
% I2 = sym('I2',[3 3]); % Inertia of link 2 in the frame of link 2 at its center of mass
I2 = [[m2*l2*l2/12+motor_mass*lc2*lc2;0;0],[0;0;0],[0;0;m2*l2*l2/12+motor_mass*lc2*lc2]];
% I3 = sym('I3',[3 3]); % Inertia of link 3 in the frame of link 3 at its center of mass
I3 = [[m3*l3*l3/12+motor_mass*lc3*lc3;0;0],[0;0;0],[0;0;m3*l3*l3/12+motor_mass*lc3*lc3]];

%% Forward Iteration: from 1 to n
T10 = exp_matrix4(-box6(A1),t(1))*M10; % Tij = Transformation matrix at any theta from frame i to frame j
T21 = exp_matrix4(-box6(A2),t(2))*M21; 
T32 = exp_matrix4(-box6(A3),t(3))*M32;

V1 = Ad(T10)*V0 + A1*t_dot(1); % Vi = Twist of link i in frame i
V2 = Ad(T21)*V1 + A2*t_dot(2);
V3 = Ad(T32)*V2 + A3*t_dot(3);

V1_dot = Ad(T10)*V0_dot + ad_twist(V1)*A1*t_dot(1) + A1*t_dotdot(1); % Vi_dot = Acceleration of link i in frame i
V2_dot = Ad(T21)*V1_dot + ad_twist(V2)*A2*t_dot(2) + A2*t_dotdot(2);
V3_dot = Ad(T32)*V2_dot + ad_twist(V3)*A3*t_dot(3) + A3*t_dotdot(3);

display(V3_dot)

%% Backward iteration: from n t1
T43 = M43;
G3 = [I3 zeros(3,3); zeros(3,3) m3*eye(3)]; % Gi = Spatial inertia matrix of link i
G2 = [I2 zeros(3,3); zeros(3,3) m2*eye(3)];
G1 = [I1 zeros(3,3); zeros(3,3) m1*eye(3)];

F3 = Ad(T43)'*F4 + G3*V3_dot - ad_twist(V3)'*G3*V3; % Fi = Wrench transferred to link i from joint i in frame i
F2 = Ad(T32)'*F3 + G2*V2_dot - ad_twist(V2)'*G2*V2;
F1 = Ad(T21)'*F2 + G1*V1_dot - ad_twist(V1)'*G1*V1;

tau3 = F3'*A3; % taui = Projection of Fi on the screw axis Ai, which should be applied by the joint i
tau2 = F2'*A2; 
tau1 = F1'*A1;

tau = [tau1;tau2;tau3]; % Joint torques vector 
display(tau)

%% Reaction force in the frame of base of the robot arm
F0 = Ad(T10)'*F1;

%% Reaction force on robot body wrt body frame
syms mbot % mass of robot 
Ibot = sym('Ibot',[3 3]); % Inertia matrix of the robot in the body frame
Gbot = [Ibot zeros(3,3); zeros(3,3) mbot*eye(3)]; % Spatial inertia matrix of the bot in body frame
T1bot = [eye(3) -(rcb+[lc1;0;0]);0 0 0 1];
Fbot = Ad(T1bot)'*F1 + Gbot*Vbot_dot - ad_twist(Vbot)'*Gbot*Vbot; % Wrench on bot due to robot arm in the body frame

%% Reaction force on robot body wrt world frame
% Fw = Ad(Tbotw)'Fbot -> for this we need transformation matrix from body
% to world, since we know R and we know p we know Tbotw