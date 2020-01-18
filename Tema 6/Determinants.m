function d = Determinants(Covar,LandObs)
%   Covar = Covariances of each landmark and the robot
%   LandObs = Vector of landmarks observed
    d=NaN*ones(length(LandObs),1);
    for i=1:length(LandObs)
        pos=LandObs(i);
        if(~isnan(pos))
            cov = Covar(pos:pos+1,pos:pos+1);
            d(i) = det(cov);
        end
    end
 %{  
%   Covar = Covariances of each landmark
%   LandObs = Number of landmarks observed * 2
d=[];
for i = 1:2:LandObs
    d=[d;det(Covar(i:i+1,i:i+1))];
end
    %}
end

