%% =================================================
% Function Q2(ptCloud)
% --------------------------------------------------
% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
%%==================================================
function [center,axis,radius] = Q2(ptCloud)
    %% Constants
    MAX_ITER1  = 1000;
    MAX_ITER2  = 10;
    EPSILON    = 0.1;
    
    %% Initialization
    nm  = pcnormals(ptCloud, 60);
    pc  = ptCloud.Location;
    pct = pc';
    mCnt = 0; i = 0;
    
    %% Calculate samples
    while(i<MAX_ITER1 || mCnt == 0)
        i = i + 1;
        
        %% Pick random 2 points
        idx  = getNRandIdx(pc, 2);
        
        %% Get surface normals
        n1   = nm(idx(1), :); - pc(idx(1), :);
        n2   = nm(idx(2), :); - pc(idx(2), :);
        
        %plotLine(n1, pc(idx(1),:), 'm');
        %plotLine(n2, pc(idx(2),:), 'm');
        
        %% Calculate orthagonal vector to surface normals
        nat = cross(n1, n2);
        nat = nat / norm(nat);
        na  = nat';
        
        %% Project points to plane orthagonal to axis
        xplane = (eye(3,3) - na*nat) * pct;
        xplane = xplane';
        %plot3(xplane(:,1),xplane(:,2),xplane(:, 3));
        
        %% Try to fit circle to projected points
        for j=1:MAX_ITER2
            %% Generate candidate cicle
            % Sample 3 points out of projected points
            pts = getNRandPts(xplane, 3);
            
            % Translate to 2D local point system
            [locx, locy, pts2D, or] = translate3Dto2D(pts);
            % Fit circle to sample points
            [nc, nr] = fitCircle(pts2D);
            % Translate to local points back to 3D system
            nc = translate2Dto3D(nc, locx, locy, or);
            
            %% Evaluate fitness
            dist = getDistance(xplane, nc);
            dist = abs(dist - nr) < EPSILON;
            cnt  = sum(dist);
            
            if(cnt> mCnt)
                center = nc';
                radius = nr;
                axis   = na';
                mCnt   = cnt;
            end;
        end;
    end;
end


%% =================================================
% Function fitCircle(pts)
% --------------------------------------------------
% Given 3 points calculates a circle that passes through those points
%
% input: pts -> 3 x 2 matrix 
%
% output: center  -> Is the center of the circle 1x2
%         radius  -> Is the radius of the circle (scalar)
%
% cite: http://www.qc.edu.hk/math/Advanced%20Level/circle%20given%203%20points.htm
%%==================================================
function [center, radius] = fitCircle(pts)
    % |(x^2 + y^2) x y 1| = 0
    mat = zeros(3,4);
    for j=1:3   
        x = pts(j,1); y = pts(j,2);
        mat(j, :) = [(x^2 + y^2) x y 1];
    end;
    
    d =  det( mat(:, 2:4));
    x = -det([mat(:, 1:1) mat(:, 3:4)]) / (2*d);
    y =  det([mat(:, 1:2) mat(:, 4:4)]) / (2*d);
    r = +det( mat(:, 1:3))              / (1*d);
    r = r + x^2 + y^2;
    
    radius = sqrt(r);
    center = [-x -y];
end



%% =================================================
% Function translate3Dto2D(pts)
% --------------------------------------------------
% Translates a planar co-ordinates from a 3D system to a local 2D system
%
% input: pts -> N x 3 matrix 
%
% output: locx  -> Is the Local translation in x(scalar)
%         locy  -> Is the Local translation in y(scalar)
%         pts2D -> Is the translated N x 2 matrix
%         origin -> 1x3 is the origin of local system 
% 
% cite: http://stackoverflow.com/questions/26369618/getting-local-2d-coordinates-of-vertices-of-a-planar-polygon-in-3d-space
%%==================================================
function [locx, locy, pts2D, origin] = translate3Dto2D(pts)
    p0 = pts(1,:); p1 = pts(2,:); p2 = pts(3,:);
    loc0 = p0;                       % local origin
    locx = p1 - loc0;                % local X axis
    normal = cross(locx, p2 - loc0); % vector orthogonal to polygon plane
    locy   = cross(normal, locx);    % local Y axis
    
    locx = locx/norm(locx);
    locy = locy/norm(locy);
    
    pts2D = zeros(size(pts, 1), 2);
    for i=1:size(pts, 1)
        p = pts(i);
        pts2D(i,:) = [dot(p - loc0, locx)    % local X coordinate
                      dot(p - loc0, locy)];  % local Y coordinate
    end;
    origin = loc0;
end


%% =================================================
% Function translate2Dto3D(pts, locx, locy, origin)
% --------------------------------------------------
% Translates a planar co-ordinates from a 3D system to a local 2D system
%
% input: pts    -> N x 3 matrix 
%        locx   -> Is the Local translation in x(scalar)
%        locy   -> Is the Local translation in y(scalar)
%        origin -> 1x3 is the origin of local system 
%
% output: pts2D -> Is the translated N x 2 matrix
%
% cite: http://stackoverflow.com/questions/26369618/getting-local-2d-coordinates-of-vertices-of-a-planar-polygon-in-3d-space
%%==================================================
function pts3D = translate2Dto3D(pts, origin, locx, locy)
    pts3D = zeros(size(pts, 1), 3);
    for i=1:size(pts, 1)
        Lx = pts(i,1); Ly = pts(i,2);
        pts3D(i,:) = origin + Lx*locx + Ly*locy;
    end;
end
