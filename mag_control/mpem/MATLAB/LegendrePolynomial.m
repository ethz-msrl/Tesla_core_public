function [ value ] = LegendrePolynomial(x, order, der )
%LegendrePolynomial provides value of a legandre polynomial of order n at
% position vector x and deritive order der. Only Reliable to the 18th order

x = reshape(x,numel(x),1);
order = reshape(order,1,numel(order));

if ~exist('der','var')
    der = 0;
end

value = zeros(length(x),length(order));
if ~isnumeric(x)
    value = sym(value);
end
for i = 1:length(order)
    n = order(i);
    
for k=der:n
     j=1:n;
     value(:,i) = value(:,i)+x.^(k-der).*prod((k-der+1):k).*nchoosek(n,k).*prod((2*j + k - n - 1)./(j));
end


if ~isnumeric(x)
    value(:,i) = simplify(value(:,i),500);
end
end



end



