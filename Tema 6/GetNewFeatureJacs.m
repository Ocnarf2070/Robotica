function [jGz] = GetNewFeatureJacs(Xv, z)
    alpha = z(2) + Xv(3);
    ca=cos(alpha); sa=sin(alpha);
    r=z(1);
    jGz = [ca -r*sa;
           sa  r*ca];
        
end    