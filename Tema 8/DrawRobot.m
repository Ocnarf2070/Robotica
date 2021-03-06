function H = DrawRobot(Xr,col,H)
    p=0.005; % percentage of axes size
    a=axis;
    l1=(a(2)-a(1))*p;
    l2=(a(4)-a(3))*p;
    P=[-1 1 0 -1; -1 -1 3 -1];%basic triangle
    theta = Xr(3)-pi/2;%rotate to point along x axis (theta = 0)
    c=cos(theta);
    s=sin(theta);
    P=[c -s; s c]*P; %rotate by theta
    P(1,:)=P(1,:)*l1+Xr(1); %scale and shift to x
    P(2,:)=P(2,:)*l2+Xr(2);
    if(isempty(H))
        H = plot(P(1,:),P(2,:),col,'LineWidth',0.1);% draw
    else
        set(H,'XData',P(1,:));
        set(H,'YData',P(2,:));
        plot(Xr(1),Xr(2),'b.','MarkerSize',10);
end
