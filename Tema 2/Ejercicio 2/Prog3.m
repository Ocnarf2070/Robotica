clear all
close all
figure
mu1=1; sigma1=1;
n_sample = 1000;
sample1 = randn(n_sample,10)*sigma1+mu1;
mu2=4; sigma2=sqrt(2);
sample2 = randn(n_sample,10)*sigma2+mu2;
sample3 = (sigma2^2*sample1+sigma1^2*sample2)/(sigma1^2+sigma2^2);
%The weighted average witch weights are sigma1 and sigma2
hold on;
[N,b]=hist(sample3,50);
bar(b,N/trapz(b,N));
x=-3:0.05:12;
plot(x,normpdf(x,mu1,sigma1),'g');
plot(x,normpdf(x,mu2,sigma2),'b');
mu3=(sigma2^2*mu1+sigma1^2*mu2)/(sigma1^2+sigma2^2);
sigma3=(sigma1^2*sigma2^2)/(sigma1^2+sigma2^2);
plot(x,normpdf(x,mu3,sqrt(sigma3)),'r');
plot(x,normpdf(x,mu1,sigma1)*normpdf(x,mu2,sigma2),'m*');
title('Weighted Average: Product of Gaussians');
hold off;