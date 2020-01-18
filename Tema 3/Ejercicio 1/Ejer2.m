% Testing the composition of robot poses
close all;
clear
nSteps = 15; %Number of motions
t = [2;0]; %translation,
ang = -90*pi/180; %orientation angle
cov=[0.04   0       0
        0   0.04    0
        0   0       0.01]; %the covariance of the gaussian error
pose_inc_straight_line = [t;0]; %pose increment
pose_inc_straight_line_and_rotation = [t;ang]; %pose increment
pose = [0;0;pi/2]; %initial pose of the robot (at k = 0)
pose_g = pose; %initial pose of the robot with the gaussian error
figure(1);hold on;axis equal;grid on;axis([-2 10 -2 10])
xlabel('x');ylabel('y'); title('Pose compotition');
DrawRobot(pose,'r'); % Draw initial pose
DrawRobot(pose_g,'b');
pause
for k = 1:nSteps %Main loop
    
    if mod(k,4) == 0
        pose = tcomp(pose,pose_inc_straight_line_and_rotation);
        mov=mvnrnd(pose_inc_straight_line_and_rotation,cov); % the gaussian error
        pose_g = tcomp(pose_g,mov');
    else
        pose = tcomp(pose,pose_inc_straight_line);
        mov=mvnrnd(pose_inc_straight_line,cov); % the gaussian error
        pose_g = tcomp(pose_g,mov');
    end
    DrawRobot(pose,'r');
    DrawRobot(pose_g,'b');
    pause
end;

function tac=tcomp(tab,tbc)
%Composition of transformations tab and tbc given by poses (i.e. vectors)
if size(tab,1) ~= 3, error('TCOMP: tab not a transformation!'); end;
if size(tbc,1) ~= 3, error('TCOMP: tbc not a transformation!'); end;
ang = tab(3)+tbc(3);
if ang > pi | ang <= -pi
   ang = AngleWrap(ang);
end
s = sin(tab(3)); c = cos(tab(3));
tac = [tab(1:2)+ [c -s; s c]*tbc(1:2); ang];
end