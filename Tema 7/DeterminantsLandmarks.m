function d = DeterminantsLandmarks(Covar,LandObs)
%   Covar = Covariances of each landmark and the robot
%   LandObs = Vector of landmarks observed
    tam = size(LandObs,1);
    d=NaN*ones(tam,1);
    for i=1:tam
        pos=LandObs(i);
        if(~isnan(pos))
            cov = Covar(pos:pos+1,pos:pos+1);
            d(i) = det(cov);
        end
    end
end

