function error = Errors(EstLand,LandObs,NLandObs)

%   EstLand = Estimated position of each landmark and the robot
%   LandObs = Vector of landmarks observed
    global Map;
    error=NaN*ones(length(LandObs),1);
    for i=1:length(LandObs)
        pos=LandObs(i);
        if(~isnan(pos))
            land=Map(:,i)';
            est=EstLand(pos:pos+1)';
            error(i)=pdist2(land,est);
        end
    end
%{
%ERRORS
%   EstLand = Estimated position of each landmark 
    LandObs = Vector of landmarks observed 
    NLandObs = Number of landmarks observed * 2
    global Map
    error=[];
    for i=1:2:NLandObs
       Pos=find(LandObs==i);
       RealLand=Map(:,Pos);
       e=EstLand(i:i+1)-RealLand;
       error=[error; e'*e];
    end
 %}
end

