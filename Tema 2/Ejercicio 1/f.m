function y=f(x,mu,sigma)
%{
In: x -> value(s)
    mu -> mean
    sigma -> standart variance
Out: y -> result for a normal distribution in x value(s)
 %}
y=(1/sqrt(2*pi*(sigma^2)))*exp(-(((x-mu).^2)/(2*sigma^2)));
end