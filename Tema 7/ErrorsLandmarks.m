function error = ErrorsLandmarks(EstLand,LandObs,Map)
%ERRORS
%   EstLand = Estimated position of each landmark and the robot
%   LandObs = Vector of landmarks observed
    tam = size(LandObs,1);
    error=NaN*ones(tam,1);
    for i=1:tam
        pos=LandObs(i);
        if(~isnan(pos))
            land=Map(:,i);
            est=EstLand(pos:pos+1);
            e=land-est;
            error(i)=e'*e;
        end
    end
end

