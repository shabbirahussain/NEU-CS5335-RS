% TODO: You write this function!
% input: f -> an 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        j3Target -> desired configuration for joint 3.
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q3(f,qInit,posGoal)
    % Constants
    qDesired = [1.5708, -0.3900, 0, 0.7850, 0, 0.7850, 0, -0.5000, 0.5000];
    alpha1 = 0.6;
    alpha2 = 0.8;
    
    % Calculate desired position
    q = qInit;
    for i=1:5
        % Calculate current positions 
        p = f.fkine(q);
        p = p(1:3,4);
        
        % Calculate current delta 
        dX = (posGoal-p);
        
        % Calculate Jacobian
        J  = f.jacob0(q, 'trans');
        Ji = pinv(J);
        
        % Calculate nullspace term
        siz = size(Ji);
        I   = eye(siz(1));
        NSp = (I - Ji * J);
        
        % Find delta Q
        dQ = (qDesired - q);
        dQ = (alpha1 * Ji * dX) + (alpha2 * NSp * dQ');
        q  = dQ' + q;
    end
end
    
    
