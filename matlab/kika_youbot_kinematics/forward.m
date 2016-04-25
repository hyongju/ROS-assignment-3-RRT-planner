%% Overview


% "Forward kinematics of KUKA Youbot," Hyongju Park (04/08/2016)
% See
% <http://www.youbot-store.com/developers/documentation/kuka-youbot-kinematics-dynamics-and-3d-model>
% to find more details on kinematics, dynamics and 3D model of KUKA YouBot

% 'stlread.m' (by MathWorks) was use to read 3D model (stl) of youbot


%% Setting robot parameters


clear all; close all; clc

% pose the robot
pos_b = [0 0];      % positions (x,y) of the robot
th_b = 0;           % orientation (yaw) of the robot

% these are the angles when arm is streched vertically
% arm joint 1, arm joint 2, arm joint 3, arm joint 4, arm joint 5
jnt_ang = [0 0 0 0 0];                  % initial configuration
%jnt_ang = [0 pi/5 pi/1.6 -pi/3 0];    % goal configuration    

% For the youbot model in ROS, the following offset angles must be added to the actual joint angles 
jnt_offset = [2.95 1.13 -2.55 1.79 2.879];

% thus the joint angles to be used in ROS is:
jnt_ang_ROS = jnt_ang + jnt_offset;

% min and max of joint angles
ang_min = [-169 -65 -151 -102.5 -165]/180*pi;
ang_max = [+169  90  146  102.5  165]/180*pi;

%% D-H parameters


% d:  offset along previous z
% th: angle about previous z
% a:  link lengths along old x to new x
% ap: link twist angles about previous x

a = [143+24 33 155 135 113.6 0 0] * 0.001;
ap = [pi pi/2 0 0 -pi/2 pi/2 0];
d = [46+84+115 0 0 0 0 0 57.16] * 0.001;
th = [0
    jnt_ang(1)
    jnt_ang(2)-pi/2 
    jnt_ang(3) 
    jnt_ang(4) 
    pi/2 
    jnt_ang(5)+pi/2]; 

%% Homogeneous transformation matrices


% world - robot
A_b = [cos(th_b) -sin(th_b) 0 pos_b(1);
    sin(th_b) cos(th_b) 0 pos_b(2);
    0 0 1 0;
    0 0 0 1];

for i=1:7
    A{i} = [cos(th(i)) -sin(th(i))*cos(ap(i)) sin(th(i))*sin(ap(i)) a(i)*cos(th(i));
        sin(th(i)) cos(th(i))*cos(ap(i)) -cos(th(i))*sin(ap(i)) a(i)*sin(th(i));
        0 sin(ap(i)) cos(ap(i)) d(i);
        0 0 0 1];
end
T01 = A_b*A{1};     % world - base frame
T02 = T01*A{2};     % world - arm base frame
T03 = T02*A{3};     % world - arm joint 1
T04 = T03*A{4};     % world - arm joint 2
T05 = T04*A{5};     % world - arm joint 3
T06 = T05*A{6};     % world - arm joint 4
T07 = T06*A{7};     % world - arm joint 5

%% Coordinates at joint origins


k = [0 0 0 1];
jnt_origin = [k' T01*k' T02*k' T03*k' T04*k' T05*k' T06*k' T07*k'];

figure,
plot3(jnt_origin(1,:),jnt_origin(2,:),jnt_origin(3,:),'-o');hold on;
axis('equal');
xlabel('x');ylabel('y');zlabel('z')

%% Reading 3D CAD models


CAD_base_frame = stlread('base_frame_convex.stl');
CAD_arm_base = stlread('arm_base_frame_convex.stl');
CAD_arm_joint_1 = stlread('arm_joint_1_convex.stl');
CAD_arm_joint_2 = stlread('arm_joint_2_convex.stl');
CAD_arm_joint_3 = stlread('arm_joint_3_convex.stl');
CAD_arm_joint_4 = stlread('arm_joint_4_convex.stl');
CAD_arm_joint_5 = stlread('arm_joint_5_convex.stl');
CAD_gripper_base = stlread('gripper_base_frame_convex.stl');
CAD_arm_joint_1_gripper_l = stlread('gripper_left_finger_convex.stl');
CAD_arm_joint_1_gripper_r = stlread('gripper_right_finger_convex.stl');

% make copies
CAD_base_frame1 = CAD_base_frame;
CAD_arm_base1 = CAD_arm_base;
CAD_arm_joint_11 = CAD_arm_joint_1;
CAD_arm_joint_21 = CAD_arm_joint_2;
CAD_arm_joint_31 = CAD_arm_joint_3;
CAD_arm_joint_41 = CAD_arm_joint_4;
CAD_arm_joint_51 = CAD_arm_joint_5;
CAD_gripper_base1 = CAD_gripper_base;
CAD_arm_joint_1_gripper_l1 = CAD_arm_joint_1_gripper_l;
CAD_arm_joint_1_gripper_r1 = CAD_arm_joint_1_gripper_r;

%% Joint axis axigned bounded box model of the youbot 


% This is the axis aligned bounded boxes based upon the provided CAD 3d
% models. Each box is originally centered at (0,0,0) of its joint
% coordinate, and is translated with the given offset value

% name, width (X) x depth(Y) x height(Z), (translation from joint coordinate's origin)
% base_frame 0.6 x 0.4 x 0.124 (offset: 0, 0, 0.022)
% arm_base: 0.22 x 0.2 x 0.09 (offset: -0.11, 0, -0.005)
% arm1: 0.2 x 0.2 x 0.12 (offset: 0, 0, 0.02)
% arm2: 0.22 x 0.08 x 0.0823 (offset: 0.0740, 0, -0.0411)
% arm3: 0.06 x 0.2 x 0.075 (offset: 0, 0.07, 0.0375)
% arm4+arm5+gripper: 0.06 x 0.02524 x 0.1 (offset: 0, 0.09346, 0)


%% Applying transformations to 3D models (vertices)


% base
for i = 1:size(CAD_base_frame.vertices,1)
    tmp_b1 = A_b*[CAD_base_frame.vertices(i,:) + [0 0 0.084] 1]';
    CAD_base_frame1.vertices(i,:) = tmp_b1(1:3,:)';
end

% arm base
for i = 1:size(CAD_arm_base.vertices,1)
    tmp_af = A_b*[CAD_arm_base.vertices(i,:) + [0.143 0 0.046+0.084] 1]';
    CAD_arm_base1.vertices(i,:) = tmp_af(1:3,:)';
end

% arm 1
for i = 1:size(CAD_arm_joint_1.vertices,1)
    tmp_a1 = T01*[cos(th(2)) -sin(th(2)) 0 0;
        sin(th(2)) cos(th(2)) 0 0;
        0 0 1 0;
        0 0 0 1]*[CAD_arm_joint_1.vertices(i,:) 1]';
    CAD_arm_joint_11.vertices(i,:)= tmp_a1(1:3,:)';
end

% arm 2
for i = 1:size(CAD_arm_joint_2.vertices,1)
    tmp_a2 = T02*[cos(th(3)) -sin(th(3)) 0 0;
    sin(th(3)) cos(th(3)) 0 0;
    0 0 1 0;
    0 0 0 1]*[CAD_arm_joint_2.vertices(i,:) 1]';
    CAD_arm_joint_21.vertices(i,:)= tmp_a2(1:3,:)';
end

% arm 3
for i = 1:size(CAD_arm_joint_3.vertices,1)
    tmp_a3 = T03*[cos(th(4)) -sin(th(4)) 0 0;
    sin(th(4)) cos(th(4)) 0 0;
    0 0 1 0;
    0 0 0 1]*[CAD_arm_joint_3.vertices(i,:) 1]';
    CAD_arm_joint_31.vertices(i,:)= tmp_a3(1:3,:)';
end

% arm 4
for i = 1:size(CAD_arm_joint_4.vertices,1)
    tmp_a4 = T04*[cos(th(5)) -sin(th(5)) 0 0;
    sin(th(5)) cos(th(5)) 0 0;
    0 0 1 0;
    0 0 0 1]*[CAD_arm_joint_4.vertices(i,:) 1]';
    CAD_arm_joint_41.vertices(i,:)= tmp_a4(1:3,:)';
end

% arm 5
for i = 1:size(CAD_arm_joint_5.vertices,1)
    tmp_a5 = T06*[cos(th(6)) -sin(th(6)) 0 0;
    sin(th(6)) cos(th(6)) 0 0;
    0 0 1 0;
    0 0 0 1]*[CAD_arm_joint_5.vertices(i,:) 1]';
    CAD_arm_joint_51.vertices(i,:)= tmp_a5(1:3,:)';
end

% gripper base
for i = 1:size(CAD_gripper_base.vertices,1)
    tmp_a6 = T07*[CAD_gripper_base.vertices(i,:) 1]';
    CAD_gripper_base1.vertices(i,:)= tmp_a6(1:3,:)';
end

% left gripper
for i = 1:size(CAD_arm_joint_1_gripper_l.vertices,1)
    tmp_gl = T07*[CAD_arm_joint_1_gripper_l.vertices(i,:)+[0 0.0082+0.0125 0] 1]';
    CAD_arm_joint_1_gripper_l1.vertices(i,:)= tmp_gl(1:3,:)';    
end

% right gripper
for i = 1:size(CAD_arm_joint_1_gripper_r.vertices,1)
    tmp_gr = T07*[CAD_arm_joint_1_gripper_r.vertices(i,:)+[0 -0.0082-0.0125 0] 1]';
    CAD_arm_joint_1_gripper_r1.vertices(i,:)= tmp_gr(1:3,:)';    
end

%% Displaying youbot 3D model


patch(CAD_base_frame1,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);        
hold on;
patch(CAD_arm_base1,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);      
hold on;
patch(CAD_arm_joint_11,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);
hold on; 
patch(CAD_arm_joint_21,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);
hold on;
patch(CAD_arm_joint_31,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);
hold on;
patch(CAD_arm_joint_41,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);
hold on;
patch(CAD_arm_joint_51,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);     
hold on;
patch(CAD_gripper_base1,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);     
hold on;     
patch(CAD_arm_joint_1_gripper_l1,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);     
patch(CAD_arm_joint_1_gripper_r1,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15,'FaceAlpha',0.5);     
light;
 
