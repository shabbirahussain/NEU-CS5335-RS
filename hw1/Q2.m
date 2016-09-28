% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
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

