%function EKF_SLAM

clear all;
close all;

% Map configuration
nFeatures = 10;
MapSize = 200;
[Map,colors] = CreateMap(MapSize,nFeatures);

%how often shall we draw?
DrawEveryNFrames = 5;
mode = 'one_landmark_in_fov';
%mode = 'landmarks_in_fov';

% Robot base characterization
SigmaX = 0.01; % Standard deviation in the x axis
SigmaY = 0.01; % Standard deviation in the y axis
SigmaTheta = 1.5*pi/180; % Bearing standar deviation
R = diag([SigmaX^2 SigmaY^2 SigmaTheta^2]); % Cov matrix

% Covariances for our very bad&expensive sensor (in the system <d,theta>)
Sigma_r = 1.1;
Sigma_theta = 5*pi/180;
Q = diag([Sigma_r,Sigma_theta]).^2;
fov = pi*2/3;
max_range = 100;

xRobot = [-MapSize/3 -MapSize/3 0]';
xRobotTrue = xRobot;

%initial conditions - no map:
xEst = xRobot;
PEst = zeros(3,3);
MappedFeatures = NaN*zeros(nFeatures,2);

% Drawings
figure(1); hold on; grid off; axis equal;
axis([-MapSize/2-40 MapSize/2+40 -MapSize/2-40 MapSize/2+40]);
for i_feat=1:nFeatures
    plot(Map(1,i_feat),Map(2,i_feat),'s','Color',colors(i_feat,:), ...
        'MarkerFaceColor',colors(i_feat,:),'MarkerSize',10);
end
set(gcf,'doublebuffer','on');
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');
DrawRobot(xEst(1:3),'g');
DrawRobot(xRobotTrue,'b');
DrawRobot(xRobot,'r');
hFOV = drawFOV(xRobotTrue,fov,max_range,'b');
PlotEllipse(xEst(1:3),PEst(1:3,1:3),5,'g');

pause;

delete(hFOV);

u = [3;0;0];

% Number of steps
nSteps = 195;
%nSteps = 1000;
turning = 50;

% Storage:
PFeatDetStore = NaN*zeros(nFeatures,nSteps);
FeatErrStore = NaN*zeros(nFeatures,nSteps);

PXErrStore = NaN*zeros(nSteps,1);
XErrStore = NaN*zeros(2,nSteps); % error in position and angle

for k = 2:nSteps
    
    %
    % Move the robot with a control action u
    %
    
    u(3) = 0;
    if (mod(k,turning)==0) u(3)=pi/2;end
    
    xRobot = tcomp(xRobot,u); % New pose without noise
    noise = sqrt(R)*randn(3,1); % Generate noise
    noisy_u = tcomp(u,noise); % Apply noise to the control action
    xRobotTrue = tcomp(xRobotTrue,noisy_u);
    
    % Useful vbles
    xVehicle = xEst(1:3);
    xMap = xEst(4:end);
    
    %
    % Prediction step
    %
    
    xVehiclePred = tcomp(xVehicle,u);
    
    PPredvv = J1(xVehicle,u)* PEst(1:3,1:3) *J1(xVehicle,u)' + J2(xVehicle,u)* R * J2(xVehicle,u)';
    PPredvm = J1(xVehicle,u)*PEst(1:3,4:end);
    PPredmm = PEst(4:end,4:end);
    
    xPred = [xVehiclePred;xMap];
    PPred = [PPredvv PPredvm;
        PPredvm' PPredmm];
    
    % Get new observation/s
    
    if strcmp(mode,'one_landmark_in_fov')
        % Get a random observations within the fov of the sensor
        [MapInFov,iFeatures] = getObservationsInsideFOV(xRobotTrue,Map,fov,max_range);
        if ~isempty(MapInFov)
            [z,iFeature] = getRandomObservationFromPose(xRobotTrue,MapInFov,Q,iFeatures);
        else
            z = [];
        end
    elseif strcmp(mode,'landmarks_in_fov')
        %
        % Point 4
        %
        [MapInFov,iFeatures] = getObservationsInsideFOV(xRobotTrue,Map,fov,max_range);
        if ~isempty(MapInFov)
            for i=1:size(MapInFov,2)
                landmark = MapInFov(:,i);
                z(:,i) = getRangeAndBearing(xRobotTrue,landmark,Q);
            end
        else
            z=[];
        end
    end
    
    %
    % Update step
    %
    
    if(~isempty(z))
        %have we seen this feature before?
        if strcmp(mode,'landmarks_in_fov')
            tam = size(MapInFov,2);
        else
            tam=1;
        end
        for i=1:tam
            if strcmp(mode,'landmarks_in_fov')
                iFeature=iFeatures(i);
            end
            if( ~isnan(MappedFeatures(iFeature,1)))
                
                %predict observation: find out where it is in state vector
                FeatureIndex = MappedFeatures(iFeature,1);
                xFeature = xPred(FeatureIndex:FeatureIndex+1);
                
                zPred = getRangeAndBearing(xVehiclePred,xFeature);
                
                % get observation Jacobians
                [jHxv,jHxf] = GetObsJacs(xVehicle,xFeature)
                
                % Fill in state jacobian
                
                %
                % Point 2, Build jH from JHxv and jHxf
                %
                jH=zeros(2,nStates-1+3);
                jH(:,1:3)=jHxv;
                jH(:,FeatureIndex:FeatureIndex+1)=jHxf;
                
                % Do Kalman update:
                Innov = z(:,i)-zPred;
                Innov(2) = AngleWrap(Innov(2));
                
                if i~=1
                    PPred=PEst;
                    xPred = xEst;
                end
                
                S = jH*PPred*jH'+Q;
                W = PPred*jH'*inv(S);
                xEst = xPred+ W*Innov;
                
                PEst = PPred-W*S*W';
                
                %ensure P remains symmetric
                PEst = 0.5*(PEst+PEst');
            else
                % this is a new feature add it to the map....
                nStates = length(xEst);
                
                xFeature = xVehiclePred(1:2) + [z(1,i)*cos(z(2,i)+xVehiclePred(3));z(1,i)*sin(z(2,i)+xVehiclePred(3))];
                if i~=1
                    xPred = xEst;
                end
                xEst = [xPred;xFeature]; %augmenting state vector
                [jGxv, jGz] = GetNewFeatureJacs(xVehicle,z(:,i));
                
                M = [eye(nStates), zeros(nStates,2);% note we don't use jacobian w.r.t vehicle
                    jGxv zeros(2,nStates-3)  , jGz];
                
                PEst = M*blkdiag(PEst,Q)*M';
                
                %remember this feature as being mapped we store its ID and position in the state vector
                MappedFeatures(iFeature,:) = [length(xEst)-1, length(xEst)];
                
            end;
        end
    else
        xEst = xPred;
        PEst = PPred;
    end;
    
    %
    % Point 3, Robot pose and features localization errors and determinants
    %
    e = xRobotTrue-xEst(1:3);
    XErrStore(:,k)=[e(1:2)'*e(1:2);
                    e(3)^2];
    PXErrStore(k)=det(PEst(1:3,1:3));
    if(length(xEst)>3)
        FeatErrStore(:,k)=ErrorsLandmarks(xEst,MappedFeatures,Map);
        PFeatDetStore(:,k)=DeterminantsLandmarks(PEst,MappedFeatures);
    end
    % Drawings
    if(mod(k-2,DrawEveryNFrames)==0)
    %if(k==nSteps)
        % Robot estimated, real, and ideal poses, fov and uncertainty
        DrawRobot(xEst(1:3),'g');
        DrawRobot(xRobotTrue,'b');
        DrawRobot(xRobot,'r');
        hFOV = drawFOV(xRobotTrue,fov,max_range,'b');
        PlotEllipse(xEst(1:3),PEst(1:3,1:3),5,'g');
        
        % A line to the observed feature
        
            
        if(~isnan(z))
            for i=1:tam
                if strcmp(mode,'landmarks_in_fov')
                    iFeature=iFeatures(i);
                end
                hLine = line([xRobotTrue(1),Map(1,iFeature)],[xRobotTrue(2),Map(2,iFeature)]);
                set(hLine,'linestyle',':');
            end;
        end
        
        % The uncertainty of each perceived landmark
        n  = length(xEst);
        nF = (n-3)/2;
        hEllipses = [];
        for i = 1:nF
            iF = 3+2*i-1;
            plot(xEst(iF),xEst(iF+1),'b*');
            hEllipse = PlotEllipse(xEst(iF:iF+1),PEst(iF:iF+1,iF:iF+1),3);
            hEllipses = [hEllipses hEllipse];
        end
        
        drawnow; % flush pending drawings events
        %pause;
        
        % Clean a bit
        delete(hFOV);
        for i=1:size(hEllipses,2)
            delete(hEllipses(i));
        end
    end
    
end

% Draw the final estimated positions and uncertainties of the features
n  = length(xEst);
nF = (n-3)/2;

for i = 1:nF
    iF = 3+2*i-1;
    plot(xEst(iF),xEst(iF+1),'cs');
    PlotEllipse(xEst(iF:iF+1),PEst(iF:iF+1,iF:iF+1),3);
end;

%
% Draw erros and determinants of the location of the robot and the
% featuers
%

figure(2); hold on;
title('Errors in robot localization');
plot(XErrStore(1,:),'b');
plot(XErrStore(2,:),'r');
legend('Error in position','Error in orientation')

figure(3); hold on;
title('Determinant of the cov. matrix associated to the robot localization');
xs = 1:nSteps;
plot(PXErrStore(:),'b');

figure(4); hold on;
title('Errors in features localization');
figure(5); hold on;
title('Log of the determinant of the cov. matrix associated to each feature');

for i=1:nFeatures
    figure(5)
    h = plot(log(PFeatDetStore(i,:)));
    set(h,'Color',colors(i,:));
    
    figure(4)
    h = plot(FeatErrStore(i,:));
    set(h,'Color',colors(i,:));
end

%end




