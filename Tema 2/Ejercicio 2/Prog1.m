clear all
close all
n_sample=1000;
Num=10000;
list=rand(n_sample,Num);
subplot(2,1,1);
[N,b]=hist(list,20);
bar(b,N/trapz(b,N));
title('Uniform R.V');
subplot(2,1,2);
[N1,b1]=hist(sum(list),50);
bar(b1/n_sample,N1);
title(sprintf('Sum of %d Uniform R.V',Num));

