% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
    % Constants
    ALPHA    = 1;       % Angular step size
    EPSILON  = 0.01;    % Error tolerance
    F_PLOT_T = false;
    
    % Add buffer space
    sphereRadius1 = sphereRadius * 1.2;
    
    % Calculate position goals
    tree=[qStart]; parent=[-1];
    qGoal = HW1_Q2(rob, qStart, xGoal);
    xGoal = rob.fkine(qGoal);
    xGoal = xGoal(1:3,4);
    scatter3(xGoal(1), xGoal(2),xGoal(3), 'filled');
    
    while(true)
        % Get a pseudo goal
        q2 = getGoal(qGoal);
        
        % Gets the closest node to expand
        idx = getNearestNeighborC(tree, q2);      % From CSpace
        %idx = getNearestNeighbor(rob, tree, q2); % From cartesian space
        
        q1  = tree(idx,:);
        
        % Get distance in configuration
        dis = pdist([q1;q2]);
        %dis = getDist(rob, q1, q2);
        
        % Don't add redundant nodes to the tree
        if(dis<EPSILON) continue; end;

        % Shortcuircit goal node
        collision = true;
        if(q2 == qGoal)
            collision = Q1(rob, q1, qGoal, sphereCenter, sphereRadius1);
        end;
        
        % Find a point towards goal if dist > alpha
        if(collision && dis > ALPHA)
            mSeg = max(40, ceil(dis/ALPHA));
            [~, w] = size(q1); q = zeros(w, mSeg);
            for i=1:length(q1)
                q(i,:) = linspace(q1(i), q2(i), mSeg);
            end
            q2 = q(:,2);
            q2 = q2';
        end

        % Check for collision
        collision = Q1(rob, q1, q2, sphereCenter, sphereRadius1);
        if(collision) continue; end;

        tree   = [tree; q2];
        parent = [parent; idx];
        
        if(F_PLOT_T) 
            p = rob.fkine(q2);
            p = p(1:3,4);
            scatter3(p(1), p(2), p(3));
        end;
        
        % Break if able to connect to goal node
        if(qGoal == q2) break; end;
    end
    
    qMilestones=[];
    [idx,~] = size(tree);
    
    while(idx ~= -1)
        qMilestones = [tree(idx,:); qMilestones];
        idx = parent(idx);
        
    end;
    %qMilestones = [qMilestones; qGoal];
    
    if(F_PLOT_T)
        [l,~] = size(qMilestones);
        for i = 1:l
            p = rob.fkine(qMilestones(i,:));
            
            scatter3(p(1,4), p(2,4), p(3,4), 'x');
        end;
    end;
    
    qMilestones
end

function goal = getGoal(qGoal)
    if(rand<.8) 
        goal = mod(rand(size(qGoal))*15, 4*pi)-2*pi;
    else 
        goal = qGoal;
    end;
end

function dis = getDist(rob, q1, q2)
    p1 = rob.fkine(q1);
    p1 = p1(1:3,4);

    p2 = rob.fkine(q2);
    p2 = p2(1:3,4);

    % Calculate distance
    dis = [p1'; p2']; 
    dis = pdist(dis);
end

function idx = getNearestNeighborC(X, p)
    [l,~] = size(X);
    dist     = X - repmat(p, l, 1);
    dist     = sqrt(sum(dist.^2,2));
    [~, idx] = min(dist);
end



function idx = getNearestNeighbor(rob, X, p)
    [l,~] = size(X);
    idx = 1; minV = +Inf;
    for i=1:l
        dist    = getDist(rob, X(i,:), p);
        if(dist<minV)
            minV = dist; idx = i;
        end;
    end;
end


function q = HW1_Q2(f,qInit,posGoal)
    % Constants
    alpha = 0.6;
    R = [1 0 0; 0 -1 0; 0 0 -1];
    
    % Calculate position goals
    q = qInit;
    posGoal = [R posGoal; 0 0 0 1];
    
    for i=1:50
        p = f.fkine(q);
        dX = tr2delta(p, posGoal);
        
        % Calculate Jacobian
        J  = f.jacob0(q);
        Ji = pinv(J);
        
        % Calculate joint angles
        dQ = alpha * Ji * dX;
        q  = dQ' + q;
    end
end
