function [MapInFov,iFeatures] = getObservationsInsideFOV(x,Map,fov,max_range)

nLandmarks = size(Map,2);
MapInFov = [];
iFeatures = [];
z = zeros(2,1);

for i_landmark = 1:nLandmarks
    Delta_x = Map(1,i_landmark) - x(1);
    Delta_y = Map(2,i_landmark) - x(2);
    
    z(1) = norm([Delta_x Delta_y]);        % Range
    z(2) = atan2(Delta_y,Delta_x) - x(3);  % Bearing
    z(2) = AngleWrap(z(2));
    
    if (z(2) < fov/2) && (z(2) > -fov/2) && (z(1) < max_range)
        MapInFov = [MapInFov Map(:,i_landmark)];
        iFeatures = [iFeatures i_landmark];
    end
end
end

