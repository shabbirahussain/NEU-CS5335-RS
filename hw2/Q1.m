% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob,q1,q2,sphereCenter,r)
    MAX_SEG = 20;
    collision = false;
    
    [~, w] = size(q1); q = zeros(w, MAX_SEG);
    for i=1:w
        q(i,:) = linspace(q1(i), q2(i), MAX_SEG);
    end
    q = q';
    
    [l, ~] = size(q);
    for i=1:l
        collision = robotCollision(rob, q(i,:),sphereCenter,r);
        if(collision) 
            break; 
        end;
    end
end
