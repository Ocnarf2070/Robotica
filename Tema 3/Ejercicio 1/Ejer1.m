% Testing the composition of robot poses
close all;
clear
nSteps = 15; %Number of motions
t = [2;0]; %translation,
ang = -90*pi/180; %orientation angle
pose_inc_straight_line = [t;0]; %pose increment
pose_inc_straight_line_and_rotation = [t;ang]; %pose increment
pose = [0;0;pi/2]; %initial pose of the robot (at k = 0)
figure(1);hold on;axis equal;grid on;axis([-2 10 -2 10])
xlabel('x');ylabel('y'); title('Testing the composition of poses');
DrawRobot(pose,'r'); % Draw initial pose
pause
for k = 1:nSteps %Main loop
    if mod(k,4) == 0
        pose = tcomp(pose,pose_inc_straight_line_and_rotation);
    else
        pose = tcomp(pose,pose_inc_straight_line);
    end
    DrawRobot(pose,'r');
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