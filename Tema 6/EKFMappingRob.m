function EKFMappingRob
clear all;
close all;

%%global variables
global Map;global nSteps;
global Reading; global ObservedTimes;

%mode = 'step_by_step';
%mode = 'visualize_process';
mode = 'non_stop';

% Num features/landmarks considered within the map
nFeatures = 5;

% Generation of the map
MapSize = 100;
Map = MapSize*rand(2,nFeatures)-MapSize/2;

% Covariances for our very bad&expensive sensor (in the system <d,theta>)
Sigma_r = 8.0;
Sigma_theta = 7*pi/180;
Q = diag([Sigma_r,Sigma_theta]).^2;

% Initial robot pose
xVehicleTrue = [-MapSize/1.5;-MapSize/1.5;0]; % We know the exact robot pose at any moment

%initial conditions - no map:
xEst = [];
PEst = []; %Covariance marix of the landmark position (2 rows per landmark)
QEst = 1.0*Q; %Covariance matrix of the measurement

% MappedFeatures relates the index of a feature from the true map and its
% place within the state (which depends on when it was observed).
MappedFeatures = NaN*zeros(nFeatures,1);

% storing the number of times a features has been seen
% also store the handler to the graphical info shown
ObservedTimes = zeros(nFeatures,2);

% Initial graphics - plot true map
figure(1); hold on; grid off;
plot(Map(1,:),Map(2,:),'g*');hold on;
axis([-MapSize-5 MapSize+5 -MapSize-5 MapSize+5]);
axis equal;
set(gcf,'doublebuffer','on'); %gcf: current figure handle
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

% Loop configuration
nSteps  = 100; % Number of motions
turning = 40; % Number of motions before turning (square path)

% Control action
u=zeros(3,1);
u(1)=(2*MapSize/1.5)/turning;
u(2)=0;

texto=[];
PFeatDetStore = NaN*zeros(nFeatures,nSteps);
FeatErrStore = NaN*zeros(nFeatures,nSteps);

for i_feat = 1:nFeatures
    colors(i_feat,:) = [rand rand rand];
end

% Start the loop!
for k = 1:nSteps
    
    %
    % Move the robot
    %
    
    u(3)=0;
    if (mod(k,turning)==0) u(3)=pi/2;end;
    
    xVehicleTrue = tcomp(xVehicleTrue,u); % Perfectly known robot pose
    
    % We assume that the map is static (the state transition model of
    xPred = xEst;
    PPred = PEst;
    
    %
    % Observe a randomn feature
    %
    
    [z,iFeature] = getRandomObservationFromPose(xVehicleTrue,Map,Q);
    
    % Update the "observedtimes" for the feature and plot the reading
    ObservedTimes(iFeature)=ObservedTimes(iFeature)+1;
    PlotNumberOfReadings(xVehicleTrue,iFeature,Map);
    
    % Have we seen this feature before?
    if( ~isnan(MappedFeatures(iFeature)) ) %Yes, it is already in the map
        
        %
        % Predict observation
        %
        
        % Find out where it is in state vector
        FeatureIndex = MappedFeatures(iFeature);
        
        % xFeature is the current estimation of the position of the
        % landmard "FeatureIndex"
        xFeature = xPred(FeatureIndex:FeatureIndex+1);
        
        % Predicts the observation
        zPred = getRangeAndBearing(xVehicleTrue,xFeature); % Hint: use getRangeAndBearing function
        
        % Get observation Jacobians
        jHxf = GetObsJacs(xVehicleTrue,xFeature);
        
        % Fill in state jacobian
        % (the jacobian is zero except for the observed landmark)
        jH=zeros(2,nStates+2);
        jH(:,FeatureIndex:FeatureIndex+1)=jHxf;
        
        if mod(k,5)==0 && nFeatures>1
            jH
        end
        %
        % Kalman update
        %
        
        Innov = z-zPred; % Innovation
        Innov(2) = AngleWrap(Innov(2));
        
        S = jH*PPred*jH'+QEst;
        K = PPred*jH'*inv(S); % Gain
        xEst = xPred+ K*Innov;
        PEst = PPred-K*S*K';
        
        %ensure P remains symmetric
        PEst = 0.5*(PEst+PEst');
        
    else % No in the current map (state)
        
        % This is a new feature, so add it to the map
        nStates = length(xEst); %dimension 2x#landmarks_in_map
        
        % The observation is in the local frame of the robot, it has to
        % be translated to the global frame
        r = z(1); alpha = z(2)+xVehicleTrue(3);
        FeatureCR=[r*cos(alpha);r*sin(alpha)];
        %xFeature = tcomp(xVehicleTrue,[FeatureCR;1]);
        %xFeature=xFeature(1:2);
        xFeature=xVehicleTrue(1:2)+FeatureCR;
        % Add it to the current state
        xEst = [xEst;xFeature]; %Each new feature two new rows
        
        % Compute the jacobian
        jGz = GetNewFeatureJacs(xVehicleTrue,z); %Dimension 2x2
        
        %M = [eye(nStates), zeros(nStates,2);% note we don't use jacobian w.r.t vehicle since the pose doesn’t have uncertainty
        %    zeros(2,nStates)  , jGz];
        
        %PEst = M*blkdiag(PEst,QEst)*M';
        %THis can also be done directly 
        PEst = [PEst                zeros(nStates,2);
                zeros(2,nStates)    jGz*QEst*jGz'];
        
        
        %remember this feature as being mapped: we store its ID for the state vector
        MappedFeatures(iFeature) = length(xEst)-1;  %Always an odd number
        
    end;
    PFeatDetStore(:,k) = Determinants(PEst,MappedFeatures);
    FeatErrStore(:,k) = Errors(xEst,MappedFeatures);
    
    
    % Drawings
    
    pause(0.005);
    
    if mod(k,5)==0
        xEst
        PEst
    end
    
    if(mod(k,2)==0)
        
        DrawRobot(xVehicleTrue,'r');%plot(xVehicleTrue(1),xVehicleTrue(2),'r*');
        DoMapGraphics(xEst,PEst,5); % Draw estimated poitns (in black) and ellipses
        %axis([-MapSize-5*(nFeatures+1) MapSize+5 -MapSize-5 MapSize+10*(nFeatures)]); % Set limits again
        axis([-MapSize-5 MapSize+5 -MapSize-5 MapSize+5]); % Set limits again
        drawnow;
        if strcmp(mode,'step_by_step')
            pause;
        elseif strcmp(mode,'visualize_process')
            pause(0.2);
        elseif strcmp(mode,'non_stop')
            % non stop!
        end
        %{
        determ=[repmat('Det=',nStates/2+1,1) num2str(Determinants(PEst,nStates+2))];
        error=[repmat('Error=',nStates/2+1,1) num2str(Errors(xEst,MappedFeatures,nStates+2))];
        x = zeros(1,nStates/2+1)-MapSize-5*(nFeatures);
        y=MapSize+10*(nFeatures-1):-10:-MapSize;
        y=y(1:nStates/2+1);
        num=[repmat('Land ',nStates/2+1,1) num2str((1:(nStates/2+1))') repmat('-> ',nStates/2+1,1)];
        str=[num determ repmat(' ',nStates/2+1,1) error];
        if(~isempty(texto))
            delete(texto);
        end
        texto = text(x,y,str);
        %}
    end;
end;

figure(2); hold on;
title('Errors in features localization');
figure(3); hold on;
title('Log of the determinant of the cov. matrix associated to each feature');
for i=1:nFeatures
    figure(3)
    h = plot(log(PFeatDetStore(i,:)));
    set(h,'Color',colors(i,:));
    
    figure(2)
    h = plot(FeatErrStore(i,:));
    set(h,'Color',colors(i,:));
end
num=[repmat('Land ',nStates/2+1,1) num2str((MappedFeatures+1)/2)];
figure(2);legend(num);
figure(3);legend(num);


end


