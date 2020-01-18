%function differential_motion_velocity

clear, close all

% PARAMETERS 
dT = 0.1;%time steps size
v = 1; % Linear Velocity 
l = 0.5; %Half the width of the robot
SigmaV = 0.1; %Standard deviation of the linear velocity. 
SigmaW = 0.1; %Standard deviation of the angular velocity
nSteps =400; %Number of motions

 
%initial knowledge (prior at k = 0)
x = [0;0;0];
xtrue = x; %Ground-truth position (unknown)
P = diag([0.2,0.4,0]); %pose covariance matrix 3x3
Q = diag([SigmaV^2 SigmaW^2]); %motion covariance matrix 2x2
 
%-------- Set up graphics -----------%
figure(1);hold on;axis equal;grid on;axis([-10 20 -5 30])
xlabel('x');ylabel('y');
title('Differential Drive model');    
 
%-------- Main loop -----------%
for k = 1:nSteps
    %control is a wiggle with constant linear velocity
    u = [v;pi/10*sin(4*pi*k/nSteps)];%[v,w]
    R = u(1)/u(2); %v/w Curvature radius
    
    %jacobians
%   sin(?(t-1))     cos(?(t-1))
    sx = sin(x(3)); cx = cos(x(3)); %sin and cos for the previous robot heading
%   sin(??(t))         cos(??(t))
    si = sin(u(2)*dT); ci = cos(u(2)*dT); %sin and cos for the heading increment
    if (u(2)==0) %linear motion w=0 --> R = infinite
        JacF_x=[1 0 -v*sx*dT;
                0 1 v*cx*dT;
                0 0 1];
        JacF_u=[cx*dT 0;
                sx*dT 0;
                0     0];
    else %Non-linear motion w=!0
        JacF_x=[1 0 R*(-sx*si-cx*(1-ci));
                0 1 R*(cx*si-sx*(1-ci));
                0 0 1];
        p1=[cx*si-sx*(1-ci) R*(cx*ci-sx*si);
            sx*si+cx*(1-ci) R*(sx*ci-cx*si);
            0               1];
        p2=[1/u(2) -v/(u(2)^2);
            0       dT];
        JacF_u=p1*p2;
    end
    %prediction steps
    P = JacF_x*P*JacF_x' + JacF_u*Q*JacF_u';
    xtrue = DifferentialModel(xtrue,u+[SigmaV ;SigmaW].*randn(2,1),dT );    
    x = DifferentialModel(x,u,dT);    
    
    
    %draw occasionally
    if(mod(k-1,20)==0)
       DrawRobot(x,'r');
       PlotEllipse(x,P,1); 
       plot(xtrue(1),xtrue(2),'ko');
    end    
end
 
 
    %------------ MODEL --------------%
function y = DifferentialModel(x,u,dT)
%This function takes pose x and transform it according to the motion u=[v,w]
%applying the differential drive model. 
% Dt: time increment
% y: Transformed pose (in world reference)
if(u(2)== 0) %linear motion w=0. Only motion in x
    %dx = u(1)*dT; dy = 0; d_thetha = 0;
    y(1,1) = x(1) + u(1)*dT*cos(x(3));
    y(2,1) = x(2) + u(1)*dT*sin(x(3));
    y(3,1) = x(3);
else %Non-linear motion w=!0
    R=u(1)/u(2); %v/w=r is the curvature radius
    y(1,1) = x(1) - R*sin(x(3)) + R*sin(x(3)+u(2)*dT);
    y(2,1) = x(2) + R*cos(x(3)) - R*cos(x(3)+u(2)*dT);
    y(3,1) = x(3) + u(2)*dT;
end
end

 
