clear all
close all
mu = [1 0];
Sigma = [3 2; 2 3];
x1 = -4:.2:6; x2 = -4:.2:4;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));

mvncdf([0 0],[1 1],mu,Sigma);
contour(x1,x2,F);
xlabel('x'); ylabel('y');
