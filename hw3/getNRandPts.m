
%% =================================================
% Function getNRandPts(iPts, n)
% --------------------------------------------------
% Gets N random samples from given list.
%
% input: iPts -> M x .. matrix 
%        n    -> number of samples to generate. Expected to be less than
%        length of first dimension of iPts.
%
% output: pts -> n x .. matrix of sample points
%%==================================================
function pts = getNRandPts(iPts, n)
    idx = getNRandIdx(iPts, n);
    
    pts = zeros(n, size(iPts,2));
    for j=1:n
        pts(j, :) = iPts(idx(j), :);
    end;
end