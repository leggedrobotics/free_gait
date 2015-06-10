clc
clear all

V = [1.0, 1.0; 0.0, 0.0; 1.1, -1.1];

% Assuming counter-clockwise ordered convex polygon.
%k = convhulln(V);
k = NaN * zeros(size(V));
for i = 1:size(V,1)
    k(i, :) = [i, mod(i, size(V,1))+1];
end

%c = mean(V(unique(k),:));
c = mean(V);

V = V - repmat(c, [size(V,1) 1]);
A  = NaN * zeros(size(k,1), size(V,2));

rc = 0;
for ix = 1:size(k,1)
    F = V(k(ix,:),:);
    if rank(F,1e-5) == size(F,1)
        rc=rc+1;
        A(rc,:)=F\ones(size(F,1),1);
    end
end

A = A(1:rc,:);
b = ones(size(A,1),1);
b = b+A*c';

A
b