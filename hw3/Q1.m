% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [center,radius] = Q1(ptCloud)
    % Constants
    MAX_ITER  = 10000;
    DELTA_ERR = 0.5;
    
    pc  = ptCloud.Location;
    max = size(pc, 1);
    
    mCnt = 0;
    for i=1:MAX_ITER
        idx = unique(randi(max, [1 6]));   % Generate sample indices
        if(length(idx)<4) continue; end;   % Failed to get 4 unique samples
        
        pts = zeros(4, 3);
        for j=1:4   
            pts(j, :) = pc(idx(j), :);
        end;
        [nc, nr] = fitSphere(pts);
        dist = getDistance(pc, nc);
        dist = abs(dist - nr) < DELTA_ERR;
        cnt  = sum(dist);
        
        if(cnt> mCnt)
            center = nc;
            radius = nr;
        end;
    end;
end

function [center, radius] = fitSphere(pts)
    % |(x^2 + y^2 + z^2) x y z 1| = 0
    mat = zeros(3,5);
    for j=1:4   
        x = pts(j,1); y = pts(j,2); z = pts(j,3); 
        mat(j, :) = [(x^2 + y^2 + z^2) x y z 1];
    end;
    
    d =  det( mat(:, 2:5));
    x = -det([mat(:, 1:1) mat(:, 3:5)]) / (2*d);
    y =  det([mat(:, 1:2) mat(:, 4:5)]) / (2*d);
    z = -det([mat(:, 1:3) mat(:, 5:5)]) / (2*d);
    r = -det( mat(:, 1:4))              / (1*d);
    r = r + x^2 + y^2 + z^2;
    
    radius = sqrt(r);
    center = [-x -y -z];
end


function dist = getDistance(X, p)
    l = size(X, 1);
    dist     = X - repmat(p, l, 1);
    dist     = sqrt(sum(dist.^2,2));
end
%normals = pcnormals(ptCloud, 20);
