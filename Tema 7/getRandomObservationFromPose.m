function [z,iFeature] = getRandomObservationFromPose(x,Map,Q,iFeatures)

nLandmarks = size(Map,2);
iFeature = randi(nLandmarks);
landmark = Map(:,iFeature);

z = getRangeAndBearing(x,landmark,Q);

if nargin == 4
    iFeature = iFeatures(iFeature);
end
end