function Potential_Fields
    clear all;
    close all;
    %clf
    % Visualization modes
    %mode = 'step_by_step';
    mode = 'non_stop';
    % Configuration of the map
    nObstacles = 75;
    MapSize = 100;
    Map = MapSize*rand(2,nObstacles);
    % Drawings
    figure(1); hold on; set(gcf,'doublebuffer','on');
    plot(Map(1,:),Map(2,:),'ro');
    xlim([0 MapSize]); ylim([0 MapSize]);
    disp('Mark the starting point')
    xStart = ginput(1)'
    disp('Mark the destination point')
    xGoal = ginput(1)'
    plot(xGoal(1),xGoal(2),'gp','MarkerSize',10);
    % Initialization of useful vbles
    xRobot = xStart;
    GoalError = xGoal - xRobot;
    Hr = DrawRobot([xRobot;0],'b',[]);
    % Algorithm configuration
    RadiusOfInfluence = 10; % Objects far away of this radius does not influence
    KGoal= 0.5; % Gain for the goal (attractive) field
    KObstacles = 250; % Gain for the obstacles (repulsive) field
    % Simulation configuration
    k = 0;
    nMaxSteps = 300;
    while(norm(GoalError)>1 && k<nMaxSteps)
        %find distance to all entities
        Dp = Map-repmat(xRobot,1,nObstacles);
        Distance = sqrt(sum(Dp.^2));
        iInfluencial = find(Distance<RadiusOfInfluence);
        %
        % Compute repulsive (obstacles) potential field
        %
        
        if(~isempty(iInfluencial))
            %
            % Point 1.1
            %
            fi=0;
            for i = iInfluencial
                ri=Distance(i);
                rmax=RadiusOfInfluence;
                di=-Dp(:,i)/norm(Dp(:,i));
                f=(1/ri-1/rmax)*1/(ri^2)*di/ri;
                fi=fi+f;
            end
            landmarks=(Map(:,iInfluencial));
            hInfluentialObstacles=scatter(landmarks(1,:),landmarks(2,:),'y*');
            Frep=KObstacles*fi;
        else %nothing close
            %
            % Point 1.1
            %
            Frep=0;
        end
        %
        % Compute attractive (goal) potential field
        % Point 1.2
        Fatt = -KGoal*-(GoalError/norm(GoalError));
        
        % Compute total (attractive+repulsive) potential field
        % Point 1.3
        FTotal = Frep+Fatt;        
        
        % New vehicle pose
        xRobot = xRobot + FTotal;
        Theta = atan2(FTotal(2),FTotal(1));
        % Drawings
        DrawRobot([xRobot;Theta],'k',Hr);
        drawnow;
        if strcmp(mode,'non_stop')
            pause(0.1); % for visibility purposes
        else
            pause;
        end
        if (~isempty(iInfluencial))
            set(hInfluentialObstacles,'Visible','off'); % handler of a plot showing a mark
            %over the obstacles that are within the radius of influence of the robot. Do this plot at
            %Point 1.1
        end
        % Update termination conditions
        GoalError = xGoal - xRobot;
        k = k+1;
    end
end

