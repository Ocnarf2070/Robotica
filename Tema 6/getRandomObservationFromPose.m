function [z,iFeature] = getRandomObservationFromPose(xVehicleTrue,Map,Q)
 
    iFeatures = size(Map,2);
    iFeature = randi(iFeatures);
    feature = Map(:,iFeature);
 
    z = getRangeAndBearing(xVehicleTrue,feature,Q);
    
end