% TODO: You write this function!
% input: f -> an 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        j3Target -> desired configuration for joint 3.
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q3(f,qInit,posGoal)
    alpha1 = 0.6;
    alpha2 = 0.8;
    q = qInit;
    
    for i=1:5
        p = f.fkine(q);
        p = p(1:3,4);
        
        dX    = (posGoal-p);
        dQDot = (qInit - q);
        
        
        J   = f.jacob0(q, 'trans');
        Jiv = pinv(J);
        I   = eye(length(Jiv));
        NSp = (I - Jiv * J);
        
        dQ  = alpha1 * Jiv * dX;
        dQ  = dQ + alpha2 * NSp * dQDot';
        q  = dQ' + q;
    end


end
    
    
