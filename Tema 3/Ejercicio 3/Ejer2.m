clear, close all

% PARAMETERS 
nSteps = 30; %Number of motions
t = [0.5;0]; %translation,
ang = -90*pi/180; %orientation angle
pose_inc_straight_line = [t;0]; %pose increment
pose_inc_straight_line_and_rotation = [t;ang]; %pose increment
a=[0.07 0.07 0.03 0.05]; %values of ai
n_particles=100;
 
%initial knowledge (prior at k = 0)
pose = [0;0;pi/2]; %initial pose of the robot (at k = 0)
truePose = pose; %Ground-truth position (unknown)
P = zeros(3,3);
Q = diag([0.04,0.04,0.01]); %pose covariance matrix 3x3
particles=zeros(3,n_particles);
particles(3,:)=pose(3);
 
%-------- Set up graphics -----------%
figure(1);hold on;axis equal;grid on;axis([-2 10 -2 10])
xlabel('x');ylabel('y'); title('Robot position from sampling');

%-------- Main loop -----------%
DrawRobot(pose,'r'); % Draw initial pose
pause
hold all
for k = 1:nSteps %Main loop
    oldPose=pose;
    oldParticles=particles;
    if mod(k,16) == 0
        pose = tcomp(pose,pose_inc_straight_line_and_rotation);
    else
        pose = tcomp(pose,pose_inc_straight_line);
    end
    for n=1:n_particles
        pt1=oldParticles(:,n);pt=particles(:,n);
        ut=OdoTransf(pose,oldPose);
        particles(:,n)=actionOdo(pt,a,ut);
        end
    if mod(k,4) == 0
        DrawRobot(pose,'r');
        scatter(particles(1,:),particles(2,:),5,'filled');
        pause
    end
    
end

function ut=OdoTransf(pt,pt1)
    xt=pt(1);yt=pt(2);ot=pt(3);
    xt1=pt1(1);yt1=pt1(2);ot1=pt1(3);
    ut=zeros(1,3);
    ut(1)=atan2(yt-yt1,xt-xt1)-ot1;
    ut(2)=sqrt((xt-xt1)^2+(yt-yt1)^2);
    ut(3)=ot-ot1-ut(1);
    ut=ut';
end

function y=actionOdo(pose,a,ut)
    ot1=ut(1);dt=ut(2);ot2=ut(3);
    a1=a(1);a2=a(2);a3=a(3);a4=a(4);
    b=a1*ot1^2+a2*dt^2;
    o1=ot1+normrnd(0,b);
    b=a3*dt^2+a4*(ot1^2+dt^2);
    d=dt+normrnd(0,b);
    b=a1*ot2^2+a2*dt^2;
    o2=ot2+normrnd(0,b);
    y=zeros(3,1);
    y(1)=pose(1)+d.*(cos(pose(3)+o1));
    y(2)=pose(2)+d.*(sin(pose(3)+o1));
    y(3)=pose(3)+o1+o2;
end
