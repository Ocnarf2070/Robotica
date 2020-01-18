function h = drawFOV(x,fov,max_range,c)

if nargin < 4; c = 'b'; end

alpha = fov/2;
angles = -alpha:0.01:alpha;
nAngles = size(angles,2);
arc_points = zeros(2,nAngles);

for i=1:nAngles
    arc_points(1,i) =  max_range*cos(angles(i));
    arc_points(2,i) =  max_range*sin(angles(i));
    
    aux_point = tcomp(x,[arc_points(1,i);arc_points(2,i);1]);
    arc_points(:,i) = aux_point(1:2);
end

h = plot([x(1) arc_points(1,:) x(1)],[x(2) arc_points(2,:) x(2)],c);

end

