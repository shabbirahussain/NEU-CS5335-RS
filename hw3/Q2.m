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
    MAX_ITER1      = 50;
    MAX_ITER_AXIS  = 100;
    MAX_ITER_CIRC  = 1000;
    EPSILON    = 0.001;
    
    %% Initialization
    nm  = pcnormals(ptCloud, 20);
    pc  = ptCloud.Location;
    pct = pc';
    inCnt = -1; i = 0;
    
    %% Calculate samples
    while(i<MAX_ITER1 || inCnt == -1)
        i = i + 1;
        
        %% Calculate axis
        nat = zeros(1, 3);
        for k=1:MAX_ITER_AXIS
            %% Pick random 2 points
            idx  = getNRandIdx(pc, 2);

            %% Get surface normals
            n1   = nm(idx(1), :);
            n2   = nm(idx(2), :);

            %plotLine(n1, pc(idx(1),:), 'm');
            %plotLine(n2, pc(idx(2),:), 'm');

            %% Calculate orthagonal vector to surface normals
            nrm = cross(n1, n2);
            nrm = nrm / norm(nrm);
            
            nat = nat + nrm;
        end;
        % Calculate average axis
        nat = nat/MAX_ITER_AXIS;axis1=nat
        %plotLine(nat, [0 0 0], 'm');
        na  = nat';
        
        %% Project points to plane orthagonal to axis
        xplane = (eye(3,3) - na*nat) * pct;
        xplane = xplane';
        %plot3(xplane(:,1),xplane(:,2),xplane(:, 3));
        
        % Resample data to thin overlaping points
        xplane1 = reSamplePts(xplane, 0.05);
        if(size(xplane1,1)<3) continue; end;
        %plot3(xplane1(:,1),xplane1(:,2),xplane1(:, 3));
        
        
        %Translate to 2D local point system
        [pts2D, locx, locy, or] = translate3Dto2D(xplane1);
        
        %% Try to fit circle to projected points
        for j=1:MAX_ITER_CIRC
            %% Generate candidate cicle
            % Sample 3 points out of projected points
            pts = getNRandPts(pts2D, 3);
            
            % Fit circle to sample points
            [nc, nr] = fitCircle(pts);
            
            % Skip iteration if radius too big
            if(isnan(nr) || nr>0.3) continue; end;
            
            % Translate to local points back to 3D system
            nc = translate2Dto3D(nc, locx, locy, or);
            
            %% Evaluate fitness
            dist = getDistance(xplane, nc);
            dist = abs(dist - nr);
            dist = dist < EPSILON;
            cnt  = sum(dist);
            
            if(cnt> inCnt)
                center = nc';
                radius = nr;
                axis   = na;
                inCnt  = cnt
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
function [pts2D, locx, locy, origin] = translate3Dto2D(pts)
    p0 = pts(1,:); p1 = pts(2,:); p2 = pts(3,:);
    
    loc0 = p0;                       % local origin
    locx = p1 - loc0;                % local X axis
    locz = cross(locx, p2 - loc0);   % vector orthogonal to polygon plane
    locy = cross(locz, locx);        % local Y axis
    
    locx = locx/norm(locx);
    locy = locy/norm(locy);
    
    pts2D = zeros(size(pts, 1), 2);
    for i=1:size(pts, 1)
        p = pts(i,:) - loc0;
        pts2D(i,:) = [dot(p, locx)    % local X coordinate
                      dot(p, locy)];  % local Y coordinate
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
function pts3D = translate2Dto3D(pts, locx, locy, origin)
    pts3D = zeros(size(pts, 1), 3);
    for i=1:size(pts, 1)
        Lx = pts(i,1); Ly = pts(i,2);
        pts3D(i,:) = origin + Lx*locx + Ly*locy;
    end;
end

%% =================================================
% Function reSamplePts(pts, minDist)
% --------------------------------------------------
% Thins the given list of points by minimum distance
% input: pts -> M x .. matrix of input points
%        minDist -> minimum distance between two samples(optional)
% output: nPts -> N x .. matrix of sample indices in range of 1..M, where
%         N<=M and distance between any two points is > minDist 
%%==================================================
function nPts = reSamplePts(pts, minDist)
    max = size(pts,1);
    nPts(1,:)= pts(randi(max),:);
    cnt = 1;
    for i=1:max
        dist = getDistance(nPts, pts(i,:));
        dist = sum(dist<minDist);
        if(dist>1) continue; end;

        cnt = cnt + 1;
        nPts(cnt,:)= pts(i,:);
    end;
end