clear all
close all
figure
mu=1; sigma=1;
n_sample = 1000;
sample = randn(n_sample ,10)*sigma+mu;
hold on
x=-4:0.05:12;
plot(x,normpdf(x,1,1),'green');
y=sample*2 + 2;
[N,b]=hist(y,150);
bar(b,N/trapz(b,N));
plot(x,normpdf(x,4,sqrt(4)),'red');
title('Linear transformation of normal distribution');
hold off

figure
mu=1; sigma=1;
n_sample = 1000;
sample= randn(1000,10)*sigma+mu;
hold on
x=-4:0.05:12;
plot(x,normpdf(x,1,1),'green');
y=sample.^2 + 2;
[N,b]=hist(y,50);
bar(b,N/trapz(b,N));
title('Nonlinear transformation of normal distribution');
hold off
