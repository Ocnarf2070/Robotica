clear all
close all
mu=2;
sigma=sqrt(2);
subplot(3,1,1);
M=100;
muestra=randn(M,1)*sigma+mu;
%This makes the random of normals follows
%the normal distribution of N(2,2)
x=-5:0.05:10;
hold on
[N,b]=hist(muestra,50);
bar(b,N/trapz(b,N));
evaluate_gaussian(x,mu,sigma);
title(sprintf('%d random samples for N(2,2)',M));
axis([-inf inf 0 0.8]);
hold off

subplot(3,1,2);
M=500;
muestra=randn(M,1)*sigma+mu;
hold on
[N,b]=hist(muestra,50);
bar(b,N/trapz(b,N));
evaluate_gaussian(x,mu,sigma);
title(sprintf('%d random samples for N(2,2)',M)); 
axis([-inf inf 0 0.5]);
hold off

subplot(3,1,3);
M=1000;
muestra=randn(M,1)*sigma+mu;
hold on
[N,b]=hist(muestra,50);
bar(b,N/trapz(b,N));
evaluate_gaussian(x,mu,sigma);
title(sprintf('%d random samples for N(2,2)',M)); 
axis([-inf inf 0 0.5]);
hold off
