%% =================================================
% Function getNRandIdx(iPts, n)
% --------------------------------------------------
% Gets N random samples indices from given list.
% input: iPts -> M x .. matrix 
%        n    -> number of samples to generate. Expected to be less than
%        length of first dimension of iPts.
%
% output: idx -> n x 1 matrix of sample indices in range of 1..M
%%==================================================
function idx = getNRandIdx(iPts, n)
    max = size(iPts, 1);
    while(true)
        idx = unique(randi(max, [1 2*n])); % Generate sample indices
        if(length(idx)>=n) break; end;     % Failed to get 4 unique samples
    end;
    
    idx = idx(1:n);
end
