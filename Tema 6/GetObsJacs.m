
function [jHxf] = GetObsJacs(xPred, xFeature)
diff=xFeature-xPred(1:2);
d=pdist2(xFeature',xPred(1:2)');
jHxf=-[-diff(1)/d   -diff(2)/d;
        diff(2)/(d^2) -diff(1)/(d^2)];
end