%% =================================================
% Function getDistance(X, p)
% --------------------------------------------------
% Given a NxK matrix and a point calculates eucledian distance of p from every point
% in X.
%
% input:  X -> A NxK matrix
% output: dist -> Nx1 matrix of eucledian distance
%%==================================================
function dist = getDistance(X, p)
    l = size(X, 1);
    dist     = X - repmat(p, l, 1);
    dist     = sqrt(sum(dist.^2,2));
end