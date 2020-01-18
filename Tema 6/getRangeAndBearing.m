function z = getRangeAndBearing(xVehicleTrue,feature,Q)
 
    Delta_x = feature(1,:) - xVehicleTrue(1);
    Delta_y = feature(2,:) - xVehicleTrue(2);
 
    z(1,:) = sqrt(Delta_x.^2 + Delta_y.^2);   % Range
    z(2,:) = atan2(Delta_y,Delta_x) - xVehicleTrue(3);  % Bearing
    z(2,:) = AngleWrap(z(2,:));
    
    if nargin == 3
        z = z + sqrt(Q)*rand(2,1); % Adding noise
    end
    
end