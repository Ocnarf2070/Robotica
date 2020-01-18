clear variables; close all; clc
% Initialization
%--------------------------------------------------------------------------
% Map/landmarks related
nLandmarks = 7;
mapSize = 140; %Size of them environment in (m)
Map = mapSize*rand(2,nLandmarks)-mapSize/2; %Landmarks uniformly distributed in the Map
% Sensor/odometry related
var_d = 0.5^2; % variance (noise) of the range measurement
R = zeros(nLandmarks); % Covariance of the observation of the landmarks
z = zeros(nLandmarks,1); % Initially. all the observations equals to zero
U = diag([100,100,1*pi/180]).^2; % Covariance of the odometry noise
% Robot pose related
xTrue = zeros(3,1); %True position, to be selected by the mouse
xEst = zeros(3,1); %Position estimated by the LSE method
xOdom = zeros(3,1); %Position given by odometry (in this case xTrue affected by noise)
% Initial graphics
figure(1); hold on; grid off; axis equal; grid on;
plot(Map(1,:),Map(2,:),'sm','LineWidth',2);hold on;
xlabel('x (m)');
ylabel('y(m)');
legend('LandMarks');
% Get the true position of the robot (ask the user)
fprintf('Please, click on the Figure where the robot is located: \n');
xTrue (1:2) = ginput(1)'; %
plot(xTrue(1),xTrue(2),'ob','MarkerSize',12)
% Set an initial guess: Where the robot believes it is (from odometry)
xOdom = xTrue + sqrtm(U)*randn(3,1);
plot(xOdom(1),xOdom(2),'+r','MarkerSize',12);
legend('LandMarks','True Position','Odo Estimation (initial guess)');
sqrt((xOdom(1)-xTrue(1))^2+(xOdom(2)-xTrue(2))^2)

% Take measurements
%--------------------------------------------------------------------------
% Get the observations to all the landmarks (data given by our sensor)
for kk = 1: nLandmarks
    % Take an observation to each landmark, i.e. compute distance to each
    % one (RANGE sensor) affected by gaussian noise
    lm = Map(:,kk)';
    pr = xTrue(1:2)';
    z(kk)=pdist2(pr,lm)+randn*sqrt(var_d);
end

% Pose estimation using Gauss-Newton for least squares optimization
%--------------------------------------------------------------------------
% Some parameters for the Gauss-Newton optimization loop
nIterations = 10; % sets the maximum number of iterations
tolerance = 0.001;% Minimum error needed for stopping the loop (convergence)
iteration = 0;
% Initialization of useful vbles
incr = ones(1,2); % Delta
jH = zeros(nLandmarks,2); % Jacobian of the observation function of all the landmarks
xEst = xOdom; %Initial estimation is the odometry position (usually noisy)
% Let's go!
while (norm(incr) > tolerance && iteration < nIterations)
    plot(xEst(1),xEst(2),'+r','MarkerSize',1 + floor((iteration*15)/nIterations));
    
    % Compute the predicted observation (from xEst) and their respective
    % Jacobians
    
    % 1) Compute distance to each landmark from xEst
    %(estimated observations)
    %predicted observations
    ez = zeros(nLandmarks,1);
    for kk = 1: nLandmarks
        lm = Map(:,kk)';
        po = xEst(1:2)';
        ez(kk)= pdist2(po,lm);
    end
    % error = difference between real observations and prediced ones.
    e = z-ez;
    residual = sqrt(e'*e); %residual error = srqt(x²+y²)
    
    % 2) Compute Jacobians with respect (x,y) (slide 13)
    % The jH is evaluated at our current guest (xEst) -> z_p
    dx=-1./ez; %% -1/dn
    z_p = xEst(1:2);
    jH=zeros(size(Map))';
    for i=1:size(Map,2)
        diff=Map(:,i)-z_p; %% (xn-x) (yn-y)
        jH(i,:)=[dx(i)*diff(1) dx(i)*diff(2)]; %%-1/dn * [(xn-x) (yn-y)]
    end
    % The observation variances R grow with the root of the distance
    R = diag(var_d*sqrt(z));
    
    % 3) Solve the equation --> compute incr
    Qinv=inv(R);
    Hf=jH'*Qinv*jH;
    gf=jH'*Qinv*e;
    incr=Hf\gf;
    
    % update position estimation
    plot([xEst(1), xEst(1)+incr(1)],[xEst(2) xEst(2)+incr(2)],'r');
    xEst(1:2) = xEst(1:2) + incr;
    fprintf('Iteration number %u residual: %1.4f [m] increment: %1.5f[m]\n',iteration+1,residual,norm(incr));
    iteration = iteration + 1;
    pause(1);
    
    
end
plot(xEst(1),xEst(2),'*g') %The last estimation is plot in green
legend('LandMarks','True Position','Odo Estimation (initial guess)');
