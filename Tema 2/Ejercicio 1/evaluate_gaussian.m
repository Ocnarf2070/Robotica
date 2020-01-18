function y = evaluate_gaussian (x,mu,sigma)
%{
In: x -> value(s)
    mu -> mean
    sigma -> standart variance
Out: y -> Object of the plot
%}
y=plot(x,f(x,mu,sigma));
end