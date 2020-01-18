%
%          Exercise of the ''Robot sensing'' lecture
%             Composition of poses and landmarks

close all
clear all
clc

% Sufix meaning:
% _w: world reference frame
% _r: robot reference frame
% Other codes:
% p: in polar coordinates
% c: in cartesian coordinates
% e.g. z1p_r represents an observation z1 in polar in the robot reference
% frame

%-------------------------------------------------------------------------%
%                               4.1.1                                     %
%-------------------------------------------------------------------------%

% Robot
p1_w = [1,2,0.5]'; % Robot R1 pose
Qp1_w = zeros(3,3); % Robot pose covariance matrix (uncertainty)
% Landmark
z1p_r = [4,0.7]'; % Measurement/observation
W1p_r = diag([0.25, 0.04]); % Sensor noise covariance

% 1. Convert polar coordinates to cartesian (in the robot frame)
r = z1p_r(1); % Useful variables
alpha = z1p_r(2);
c = cos(alpha);
s = sin(alpha);

z1xc_r = r*c;
z1yc_r = r*s;
zc_r = [z1xc_r, z1yc_r];

% 2. Obtain the sensor/measurement covariance in cartesian coordinates in
% the frame of the robot (it is given in polar). For that you need the 
% Jacobian built from the expression that converts from polar to cartesian
% coordinates. 

J_pc = [c -r*s;
        s  r*c]; % Build the Jacobian

Wzc_r = J_pc*W1p_r*J_pc';

% 3. Ok, we are now ready for computing the sensor measurement in the 
% world's coordinate system (mean and covariance).

z1_w = tcomp(p1_w,[zc_r';1]) % Compute coordinates of the landmark in the world

axp=zc_r(1); ayp=zc_r(2);
theta = p1_w(3);
co = cos(theta);
so = sin(theta);

J_ap = [1 0 -axp*so-ayp*co;
        0 1  axp*co-ayp*so]; % Now build the Jacobians 
J_aa = [co -so;
        so  co];

Wzc_w = J_ap*Qp1_w*J_ap' + J_aa*Wzc_r*J_aa' % Finally, propagate the covariance!

% Draw results
plot(z1_w(1),z1_w(2),'x');
xlim([-1,10])
ylim([-1,10])
grid on;
hold on;
pbaspect([1 1 1])
text(z1_w(1)+1,z1_w(2),'Landmark','color','k');
PlotEllipse(z1_w(1:2),Wzc_w,1,'m');
DrawRobot(p1_w,'b');
text(p1_w(1)+1,p1_w(2),'R1','color','b');

%-------------------------------------------------------------------------%
%                               4.1.2                                     %
%-------------------------------------------------------------------------%

% Now, we have uncertainty in the robot pose!
Qp1_w = diag([0.08,0.6, 0.02]); % New R1 covariance diag(x, y, theta)

% Propagate the covariances again, taken into account this new info.
Wzc_w = J_ap*Qp1_w*J_ap' + J_aa*Wzc_r*J_aa' 

% Draw result
PlotEllipse(p1_w(1:2),Qp1_w,1,'b');
PlotEllipse(z1_w(1:2),Wzc_w,1,'b');

%-------------------------------------------------------------------------%
%                               4.1.3                                     %
%-------------------------------------------------------------------------%
p2_w  = [6,4,2.1]'; % Pose of the second robot R2
Qp2_w = diag([0.20,0.09,0.03]); % Covariance matrix related to the pose

% Draw robot
PlotEllipse(p2_w(1:2),Qp2_w,1,'g');
DrawRobot(p2_w,'g');
text(p2_w(1)+1,p2_w(2),'R2','color','g');

% Compute the relative pose between p12 between R1 and R2
% First way: composition of poses with inverse pose
p1inv_w = tinv(p1_w);
p12_w = tcomp(p1inv_w,p2_w)

JacInv = Jinv(p1_w);
Qinvp1_w = JacInv*Qp1_w*JacInv';

JacP12Inv=J1(p1inv_w,p2_w);
JacP12P2=J2(p1inv_w,0);
                                                          
Qp12_w = JacP12Inv*Qinvp1_w*JacP12Inv'+JacP12P2*Qp2_w*JacP12P2'

% Second way: Inverse Composition
c = cos(p1_w(3)); % Useful variables
s = sin(p1_w(3));
xp1 = p1_w(1); yp1 = p1_w(2);
xp2 = p2_w(1); yp2 = p2_w(2);

J_p12p1 =[-c -s -(xp2-xp1)*s+(yp2-yp1)*c
           s -c -(xp2-xp1)*c-(yp2-yp1)*s
           0  0 -1];
         
J_p12p2 = [c  s 0;
          -s  c 0;
           0  0 1];
    
Qp12_w = J_p12p1*Qp1_w*J_p12p1'+J_p12p2*Qp2_w*J_p12p2'


%-------------------------------------------------------------------------%
%                               4.1.4                                     %
%-------------------------------------------------------------------------%

% 1. Take a measurement using the range-bearing observation model!
r2 = sqrt(sum((z1_w(1:2)-p2_w(1:2)).^2));
xi=z1_w(1);yi=z1_w(2);
x=p2_w(1);y=p2_w(2);
alpha2 = atan2((yi-y),(xi-x))-p2_w(3);
z2p_r = [r2,alpha2]'

% 2. Jacobian from cartesian to polar at z2p_r when the covariance is in 
% global coordianes
alpha = alpha2 + p2_w(3);
ca=cos(alpha); sa=sin(alpha);
Jcp_p2 = [ca    sa;
         -sa/r2 ca/r2]; 

% 3. Finally, propagate the uncertainty to polar coordinates in the 
% robot frame 
W2_p = Jcp_p2*Wzc_w*Jcp_p2'  %dim: 2x2




%-------------------------------------------------------------------------%
%                               4.1.5                                     %
%-------------------------------------------------------------------------%
z2p_r = [4,0.3]';
W2p_r = diag([0.25, 0.04]); % Sensor noise covariance

% 1. Convert polar coordinates to cartesian (in the robot frame)
r = z2p_r(1); % Useful variables
alpha = z2p_r(2);
c = cos(alpha);
s = sin(alpha);

x2 = r*c;
y2 = r*s;
z2c_r = [x2,y2];


% 2. Obtain the sensor/measurement covariance in cartesian coordinates in
% the frame of the robot (it is given in polar). For that you need the 
% Jacobian built from the expression that converts from polar to cartesian
% coordinates. 
r = z2p_r(1);
alpha = z2p_r(2);
c = cos(alpha);
s = sin(alpha);

J_pc = [c -r*s;
        s  r*c];

Wz2c_r = J_pc*W2p_r*J_pc';

% 3. Ok, we are now ready for computing the sensor measurement in the 
% world's coordinate system (mean and covariance).

axp=z2c_r(1); ayp=z2c_r(2);
theta = p2_w(3);
co = cos(theta);
so = sin(theta);

J_ap = [1 0 -axp*so-ayp*co;
        0 1  axp*co-ayp*so];
J_aa = [co -so;
        so  co];

Wz2c_w = J_ap*Qp2_w*J_ap' + J_aa*Wz2c_r*J_aa'
z2_w = tcomp(p2_w,[z2c_r';1]) % Compute coordinates of the landmark in the world

% Draw result
plot(z2_w(1),z2_w(2),'xg');
PlotEllipse(z2_w(1:2),Wz2c_w,1,'g');

% 4. Combine the measurements from both sensors!
Wz_w = inv(inv(Wzc_w)+inv(Wz2c_w))
z_w = Wz_w*(Wzc_w\z1_w(1:2)+Wz2c_w\z2_w(1:2))

% Draw result
plot(z_w(1),z_w(2),'xr');
PlotEllipse(z_w(1:2),Wz_w,1,'r');

%-------------------------------------------------------------------------%
