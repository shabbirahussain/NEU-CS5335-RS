%% =================================================
% Function Q1(ptCloud)
% --------------------------------------------------
% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
%
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
%%==================================================
function [center,radius] = Q1(ptCloud)
    %% Constants
    MAX_ITER  = 1000;
    EPSILON   = 0.001;
    
    %% Initialization
    pc  = ptCloud.Location;
    inCnt = -1;
    
    %% Calculate samples
    for i=1:MAX_ITER
        %% Pick random 4 points
        pts = getNRandPts(pc, 4);
        
        %% Generate candidate sphere
        [nc, nr] = fitSphere(pts);
        
        %% Evaluate fitness
        dist = getDistance(pc, nc);
        dist = abs(dist - nr) < EPSILON;
        cnt  = sum(dist);
        
        if(cnt> inCnt)
            center = nc;
            radius = nr;
            inCnt  = cnt
        end;
    end;
end

%% =================================================
% Function fitSphere(pts)
% --------------------------------------------------
% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
%
% input: pts -> 4x3 matrix containing 4 3D points to fit
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
%%==================================================
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