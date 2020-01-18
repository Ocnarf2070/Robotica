clear all
close all 
min=-18; max=18; step=0.05;
x=min:step:max;
type='same';
con = conv(normpdf(x,1,1),normpdf(x,4,sqrt(2)),type)*step;
l1=normpdf(x,1,1); l2 = normpdf(x,4,2);
if(strcmp(type,'same')) 
    x1=min:step:max; 
else
    x1=min:step/2:max; 
end
hold all
plot(x1,con,'red');
plot(x,normpdf(x,5,sqrt(3)),'blue');
