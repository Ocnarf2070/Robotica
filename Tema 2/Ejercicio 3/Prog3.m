clear all
close all
n_samples = 500; 
mean = [1 0]'; sigma = [3 2; 2 3];
A = [-1 2; 2 1.5]; 
b = [3 0]';
sample=mvnrnd(mean,sigma,n_samples);
sample_l=sample*A+b';
hold on
scatter(sample(:,1),sample(:,2),'+','g');
scatter(sample_l(:,1),sample_l(:,2),10,'filled','s','m');
PlotEllipse(mean,sigma,2,'g');
PlotEllipse(A*mean+b,A*sigma*A',2,'m');
axis([-15 20 -inf inf]);
title('Linear tranformation of normal distributions');
hold off
