clear, close all

% PARAMETERS 
nSteps = 15; %Number of motions
t = [2;0]; %translation,
ang = -90*pi/180; %orientation angle
pose_inc_straight_line = [t;0]; %pose increment
pose_inc_straight_line_and_rotation = [t;ang]; %pose increment
 
%initial knowledge (prior at k = 0)
pose = [0;0;pi/2]; %initial pose of the robot (at k = 0)
truePose = pose; %Ground-truth position (unknown)
P = zeros(3,3);
Q = diag([0.04,0.04,0.01]); %pose covariance matrix 3x3
 
%-------- Set up graphics -----------%
figure(1);hold on;axis equal;grid on;axis([-2 10 -2 10])
xlabel('x');ylabel('y'); title('Differential motion with odometry commands');

%-------- Main loop -----------%
DrawRobot(pose,'r'); % Draw initial pose
pause
hold all
for k = 1:nSteps %Main loop
    oldPose=truePose;
    if mod(k,4) == 0
        pose = tcomp(pose,pose_inc_straight_line_and_rotation);
        mov=mvnrnd(pose_inc_straight_line_and_rotation,Q);
        truePose = tcomp(truePose,mov');
    else
        pose = tcomp(pose,pose_inc_straight_line);
        mov=mvnrnd(pose_inc_straight_line,Q);
        truePose = tcomp(truePose,mov');
    end
    P=DiffMotOdo(P,Q,oldPose,mov);
    DrawRobot(pose,'r');
    PlotEllipse(pose,P,0.5); 
    plot(truePose(1),truePose(2),'ko');
    pause
end;
 
function sigma=DiffMotOdo(covx,covu,oldPose,u)
JacX=J1(oldPose,u);
JacU=J2(oldPose);
sigma=JacX*covx*JacX'+JacU*covu*JacU';
end
