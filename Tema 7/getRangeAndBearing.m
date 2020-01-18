function z = getRangeAndBearing(x,landmark,Q)

Delta_x = landmark(1,:) - x(1);
Delta_y = landmark(2,:) - x(2);

z(1,:) = sqrt(Delta_x.^2 + Delta_y.^2);  % Range
z(2,:) = atan2(Delta_y,Delta_x) - x(3);  % Bearing

if nargin == 3
    z = z + sqrt(Q)*rand(2,1); % Adding noise
end

z(2,:) = AngleWrap(z(2,:));

end

