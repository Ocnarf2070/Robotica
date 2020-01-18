clear all
close all
n_samples = 500; 
mean1 = [1 0]'; sigma1 = [3 2; 2 3]; 
mean2 = [2 3]'; sigma2 = [2 0; 0 1];
sample1=mvnrnd(mean1,sigma1,n_samples);
sample2=mvnrnd(mean2,sigma2,n_samples);
hold all
scatter(sample2(:,1),sample2(:,2),5,'filled');
scatter(sample1(:,1),sample1(:,2),'+','g');
PlotEllipse(mean2,sigma2,2,'b');
PlotEllipse(mean1,sigma1,2,'g');
sigmaprod_1=inv(sigma1)+inv(sigma2);
meanprod=sigmaprod_1\(sigma1\mean1+sigma2\mean2);
sigmaprod=inv(sigmaprod_1);
PlotEllipse(meanprod,sigmaprod,4,'m');
title('Distribution of two Weighted Average random variable (Blue and Green)');
hold off
axis([-6 10 -inf inf])