clear all
close all
n_samples = 500; 
mean1 = [1 0]'; sigma1 = [3 2; 2 3]; 
mean2 = [2 3]'; sigma2 = [2 0; 0 1];
sample1=mvnrnd(mean1,sigma1,n_samples);
sample2=mvnrnd(mean2,sigma2,n_samples);
figure
hold on
scatter(sample2(:,1),sample2(:,2),10,'filled','s');
scatter(sample1(:,1),sample1(:,2),'+','g');
PlotEllipse(mean2,sigma2,2,'b');
PlotEllipse(mean1,sigma1,2,'g');
sample3=sample1+sample2;
PlotEllipse(mean1+mean2,sigma1+sigma2,2,'m');
axis([-6 10 -inf inf])
title('Distribution of Sum of two random variable (Blue and Green)');
hold off
figure
scatter(sample3(:,1),sample3(:,2),'*','m');
PlotEllipse(mean2,sigma2,2,'b');
PlotEllipse(mean1,sigma1,2,'g');
PlotEllipse(mean1+mean2,sigma1+sigma2,2,'m');
axis([-6 10 -inf inf])
title('Distribution of Sum of two random variable');

