% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob,qMilestones,sphereCenter,sphereRadius)
    qMilestonesSmoothed = qMilestones;
    
    [pGoal,~] = size(qMilestonesSmoothed);
    while(true)
        [l,~] = size(qMilestonesSmoothed);
        qGoal = qMilestonesSmoothed(pGoal,:);
        for i=1:l-2
            q1 = qMilestonesSmoothed(i,:);
            % Keep on moving source till we find unobstructed path
            collision = Q1(rob, q1, qGoal, sphereCenter, sphereRadius);
            if(collision) continue; end;
            
            % Delete intermediate rows
            qMilestonesSmoothed(i+1:pGoal-1,:) = [];
            pGoal = i+1; 
            break;
        end;
        
        % Set new goal
        pGoal = pGoal - 1;
        if(pGoal<3) break; end;
    end;
    qMilestonesSmoothed
end
