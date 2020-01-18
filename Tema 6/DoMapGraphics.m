function DoMapGraphics(xMap,PMap,nSigma)
 
    persistent k;
    persistent handler_ellipse; %%cga animating ellipses
    persistent handler_state; %%cga animating ellipses
 
    if(isempty(k))
        k = 0;
    end;
    k = k+1;
 
    % removing ellipses from the previous iteration
    if isempty(handler_ellipse)
        handler_ellipse=zeros(length(xMap));
    else
        for i=1:length(handler_ellipse)
            if (handler_ellipse(i)~=0)
                delete (handler_ellipse(i));
            end
        end
    end
    
    % removing state from the previous iteration
    if (isempty(handler_state))
        handler_state=zeros(length(xMap));
    else
        for i=1:length(handler_state)
            if (handler_state(i)~=0)
                delete (handler_state(i));
            end
        end
    end
    
    handler_ellipse=zeros(length(xMap));
    handler_state=zeros(length(xMap));
 
    colors = 'kkkk';
    
    for i = 1:length(xMap)/2
        iL = 2*i-1; iH = 2*i;
        x = xMap(iL:iH);
        P = PMap(iL:iH,iL:iH);    
        handler_ellipse(i)= PlotEllipse(x,P,nSigma,'k');        
        handler_state(i)= plot(x(1),x(2),'k.');  
        c = colors(mod(i,4)+1);
        set(handler_ellipse(i),'color',char(c));
       % plot3(x(1),x(2),k,'r+');
    end
end