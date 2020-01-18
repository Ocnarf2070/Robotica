
function EKFLocalization
clear; close all;
 
% Map configuration
Size = 50;
NumLandmarks = 10;
Map=CreateMap(NumLandmarks, Size);  % Create map of size [Size*2 Size*2]
 
%mode = 'one_landmark';
%mode = 'one_landmark_in_fov';
mode = 'landmarks_in_fov';
 
% Sensor characterization
SigmaR = 1; % Standard deviation of the range 
SigmaB = 0.7; % Standard deviation of the bearing
Q = diag([SigmaR^2 SigmaB^2]); % Cov matrix
fov = pi/2;        % field of view = 2*alpha
max_range = Size;  % maximum sensor measurement range
 
% Robot base characterization
SigmaX = 0.8; % Standard deviation in the x axis
SigmaY = 0.8; % Standard deviation in the y axis
SigmaTheta = 0.1; % Bearing standar deviation
R = diag([SigmaX^2 SigmaY^2 SigmaTheta^2]); % Cov matrix
 
% Initialization of poses 
x = [-Size+Size/3 -Size+Size/3 pi/2]';      % Ideal robot pose
xTrue = [-Size+Size/3 -Size+Size/3 pi/2]';  % Real robot pose
xEst = [-Size+Size/3 -Size+Size/3 pi/2]';   % Estimated robot pose by EKF
sEst = zeros(3,3);                          % Uncertainty of estimated robot pose
 
% Drawings
plot(Map(1,:),Map(2,:),'sm');
axis([-Size-5 Size+5 -Size-5 Size+5]);
hold on;
DrawRobot(x,'r');    
DrawRobot(xTrue,'b');
DrawRobot(xEst,'g');
PlotEllipse(xEst,sEst,4,'g');
 
nSteps = 20; % Number of motions
turning = 5; % Number of motions before turning (square path)
 
u = [(2*Size-2*Size/3)/turning;0;0]; % Control action
 
pause;
 
% Let's go!
for k = 1:nSteps-3 % Main loop
   
    u(3) = 0;
    if mod(k,turning) == 0 % Turn?
        u(3) = -pi/2;
    end
    
    x = tcomp(x,u); 			% New pose without noise    
    noise = sqrt(R)*randn(3,1); 	% Generate noise
    noisy_u = u+noise; 			% Apply noise to the control action        
    xTrue = tcomp(xTrue,noisy_u);
    % New noisy pose (real robot pose)   
    
    MapInFov=[0;0];
    % Get sensor observation/s
    if strcmp(mode,'one_landmark')
        [z,landmark] = getRandomObservationFromPose(xTrue,Map,Q);
        x0=xTrue(1);y0=xTrue(2);
        x1=landmark(1); y1=landmark(2);
        line([x0, x1],[y0, y1],'LineStyle','--');
    elseif strcmp(mode,'one_landmark_in_fov')
		MapInFov = getLandmarksInsideFOV(xTrue,Map,fov,max_range);
        if size(MapInFov,1)~=0
            [z,landmark] = getRandomObservationFromPose(xTrue,MapInFov,Q);
            x0=xTrue(1);y0=xTrue(2);
            x1=landmark(1); y1=landmark(2);
            line([x0, x1],[y0, y1],'LineStyle','--');
        end
    elseif strcmp(mode,'landmarks_in_fov')
        MapInFov = getLandmarksInsideFOV(xTrue,Map,fov,max_range);
        if size(MapInFov,1)~=0
            n=size(MapInFov,2);
            z=zeros(2*n,1);
            k=1;
            for i=1:n
                landmark=MapInFov(:,i);
                zn=getRangeAndBearing(xTrue,landmark);
                z(k)=zn(1);
                z(k+1)=zn(2);
                k=k+2;
                x0=xTrue(1);y0=xTrue(2);
                x1=landmark(1); y1=landmark(2);
                line([x0, x1],[y0, y1],'LineStyle','--');
            end
        end
    end

    %
    % EKF Localization
    %
    
    % Prediction
    jG=J1(xEst,u);
    j2=J2(xEst,u);
    Rt=j2*R*j2';
    PredU = tcomp(xEst,u);
    PredS = jG*sEst*jG'+Rt;
    xEst=PredU;
    sEst=PredS;
    
    if size(MapInFov,1)~=0
        % Correction (You need to compute the gain k and the innovation z-z_p)
        if strcmp(mode,'landmarks_in_fov')
            jH=getObsJac(PredU,[],MapInFov);
            Kt = PredS*jH'/(jH*PredS*jH'+diag(repmat(diag(Q),n,1)));
            n=size(MapInFov,2);
            hu=zeros(2*n,1);
            k=1;
            for i=1:n
                hun=getRangeAndBearing(PredU,MapInFov(:,i));
                hu(k)=hun(1);
                hu(k+1)=hun(2);
                k=k+2;
            end
        else
            jH=getObsJac(PredU,landmark);
            Kt = PredS*jH'/(jH*PredS*jH'+Q);
            hu=getRangeAndBearing(PredU,landmark);
        end
        xEst = PredU + Kt*(z-hu);
        sEst = (eye(3)-Kt*jH)*PredS;
    end
    
    % Drawings
    % Plot the FOV of the robot
    if strcmp(mode,'one_landmark_in_fov') || strcmp(mode,'landmarks_in_fov')
        h = drawFOV(xTrue,fov,max_range,'g');
    end
    
    % Plot Robot pose and uncertainty
    DrawRobot(x,'r');       % Ideal Pose (noise free)
    DrawRobot(xTrue,'b');   % Real pose (noisy)
    DrawRobot(xEst,'g');    % EKF estimation of the pose (motion+obs)
    PlotEllipse(xEst,sEst,3,'g');   %Uncertainty of EKF estimation 
    
    %pause
    pause(0.5);
    %drawnow;
 
    %Delete the previous FOV
    if strcmp(mode,'one_landmark_in_fov') || strcmp(mode,'landmarks_in_fov')
        delete(h);
    end  
    
end;
 
end % main
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function Map=CreateMap(NumLandmarks, Size)
 
Map=Size*2*rand(2,NumLandmarks)-Size;
 
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [z,landmark] = getRandomObservationFromPose(x,Map,Q)
    pos = randi(size(Map,2));
    landmark=Map(:,pos);
    z=getRangeAndBearing(x,landmark,Q);
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function z = getRangeAndBearing(x,landmark,Q)
 
    d=pdist2(landmark(1:2)',x(1:2)');
    xi=landmark(1); yi=landmark(2);
    angle=atan2(yi-x(2),xi-x(1))-x(3);
    z=[d;angle];
    if nargin == 3 % Add noise
       z=z+sqrt(Q)*randn(2,1); 
    end

% utilize AngleWrap to ensure that the measurement angle is correct
z(2)=AngleWrap(z(2));
    
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
function jH = getObsJac(xPred,Landmark, Map)
         if nargin == 3
            n=size(Map,2);
            jH=zeros(2*n,3);
            k=1;
            for i=1:n
                jh=getObsJac(xPred,Map(:,i));
                jH(k:k+1,:)=jh;
                k=k+2;
            end
         else
            diff=Landmark-xPred(1:2);
            d=pdist2(Landmark',xPred(1:2)');
            jH=[-diff(1)/d   -diff(2)/d      0;
                 diff(2)/d^2 -diff(1)/d^2   -1];
         end
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function MapInFov = getLandmarksInsideFOV(x,Map,fov,max_range)
	%{
    max_range -> max distance from the robot
    fov -> angle of vision
    Map -> THE Map
    x -> position and angle of the robot
    %}
    cont=1;
    MapInFov=[];
    alpha = fov/2;
    min_angle=x(3)-alpha;
    max_angle=x(3)+alpha;
    nLandmark=length(Map);
    for i=1:nLandmark
        landmark=Map(:,i);
        z=getRangeAndBearing(x,landmark);
        angle=z(2)+x(3);
        dist=z(1);
        if dist<=max_range && angle<=max_angle && angle>=min_angle
            MapInFov(:,cont)=landmark;
            cont=cont+1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function h = drawFOV(x,fov,max_range,c)
 
    if nargin < 4; c = 'b'; end
    
    alpha = fov/2;
    angles = -alpha:0.01:alpha;
    nAngles = size(angles,2);
    arc_points = zeros(2,nAngles);
    
    for i=1:nAngles
       arc_points(1,i) =  max_range*cos(angles(i));
       arc_points(2,i) =  max_range*sin(angles(i));
 
       aux_point = tcomp(x,[arc_points(1,i);arc_points(2,i);1]);
       arc_points(:,i) = aux_point(1:2);
    end
 
    h = plot([x(1) arc_points(1,:) x(1)],[x(2) arc_points(2,:) x(2)],c);
 
end
