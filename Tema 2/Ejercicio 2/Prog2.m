clear all
close all
figure
mu=1; sigma=1;
n_sample=1000;
sample1 = randn(n_sample,10)*sigma+mu;
mu=4; sigma=sqrt(2);
sample2 = randn(n_sample,10)*sigma+mu;
sample3 = sample1 + sample2;
hold on
[N,b]=hist(sample3,50);
bar(b,N/trapz(b,N));
x=-5:0.05:15;
%normpdf is a native funtion on Matlab with calcules the pdf
%of a normal distribution in x with a mean and a typical deviation
plot(x,normpdf(x,5,sqrt(3)),'red');
plot(x,normpdf(x,1,1),'green');
plot(x,normpdf(x,4,sqrt(2)),'blue');
title('Sum of R.V.');
hold off


figure
mn=-18; mx=18; st=0.05;
x=mn:st:mx;
con = conv(normpdf(x,1,1),normpdf(x,4,sqrt(2)),'same')*st;
%We multiply here with the step to normalize the results
hold on
plot(x,con,'red');
plot(mn:0.5:mx,normpdf(mn:0.5:mx,5,sqrt(3)),'*');
title('Convolution of R.V');
hold off
