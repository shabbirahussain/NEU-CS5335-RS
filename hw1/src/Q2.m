% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
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




% Extra: Kindly ignore
function q = Q2_1(f,qInit,posGoal)
    alpha = 0.6;
    q = qInit;
    
    for i=1:5
        p = f.fkine(q);
        p = p(1:3,4);
        
        deltaX  = (posGoal-p);

        J  = f.jacob0(q, 'trans');
        deltaQ  = alpha * pinv(J) * deltaX;
        q  = deltaQ' + q;
    end
end