% Title : Force rotation matrices to a particular form if possible
% Author: Shreyash Annapureddy
% Date  : 14/03/2016

function [T] = ConstrainTransformations(T)

[~,~,num] = size(T);

for j = 1:num
    constraint = diag(T(:,:,j));
    constraint = abs(constraint(1:3));
    constraint = sum(constraint);
    if 3 - constraint <= 0.24
        T(1:3,1:3,j) = eye(3);
    end
end

end