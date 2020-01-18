function PlotNumberOfReadings(xVehicleTrue,iFeature,Map)
 
    global Reading;
    global ObservedTimes;
 
    for c=1:length(Reading)
       if (~isnan(Reading(c))) 
           delete(Reading(c));
       end; 
    end
 
    Reading=zeros(length(iFeature));
 
    if (ObservedTimes(iFeature,2)~=0) delete(ObservedTimes(iFeature,2));end;
    ObservedTimes(iFeature,2)=text(Map(1,iFeature)+rand(), ...
        Map(2,iFeature)+rand(),sprintf('%d',ObservedTimes(iFeature,1)));
 
    for c=1:length(iFeature)
       if (iFeature(c)~=-1)
        Reading(c)=line([xVehicleTrue(1), Map(1,iFeature(c))], ...
                        [xVehicleTrue(2), Map(2,iFeature(c))]);
       else
           Reading(c)=NaN;
       end
    end
end