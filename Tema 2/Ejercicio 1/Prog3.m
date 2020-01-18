clear all
close all
mu=2;
sigma=sqrt(2);
x=randn(1,200)*mu+sigma;
%This makes the random of normals follows
%the normal distribution of N(2,2)
y=zeros(size(x));
scatter(x,y,10,'filled','d');

    