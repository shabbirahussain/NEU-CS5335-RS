load fisheriris;

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
    alpha = 0.001;
    
    tree=[];
    
    goal = getGoal(xGoal);
    
    
    
    
    
    qMilestones=[];
end

function goal = getGoal(xGoal)
    if(rand<.9) 
        goal = rand(3, 1);
    else 
        goal = xGoal;
    end;
end;