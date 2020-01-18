function [jGx,jGz] = GetNewFeatureJacs(Xv, z)

theta = Xv(3,1);
r = z(1);
bearing = z(2);
jGx = [ 1   0   -r*sin(theta + bearing);
    0   1   r*cos(theta + bearing)];
jGz = [ cos(theta + bearing) -r*sin(theta + bearing);
    sin(theta + bearing) r*cos(theta + bearing)];
end
