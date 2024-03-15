function inter = intersection(x,y)
%INTERSECTION Summary of this function goes here
%   Detailed explanation goes here
kernel = null([x y],1e-4);
p1 = size(x,2);
p2 = size(y,2);

P_1 = kernel(1:p1,:);
P_2 = kernel(p1+1:size(kernel,1),:);
inter = x*P_1;
end

