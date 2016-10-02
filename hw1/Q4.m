% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q4(f1,f2,qInit,f1Target,f2Target)
    alpha = 0.05;
    q = qInit(:,1:7);
    R = [1 0 0; 0 -1 0; 0 0 -1];
    posGoal = [R f1Target; 0 0 0 1];
    
    for i=1:50
        p = f1.fkine([q 0 0]);
        %deltaX  = (posGoal-p);
        %deltaX = tr2delta(deltaX);
        
        deltaX = tr2delta(p, posGoal);
        
        J  = f1.jacob0([q 0 0]);
        deltaQ  = alpha * pinv(J) * deltaX;
        q  = deltaQ(1:7,:)' + q ;
    end
    q = [q 0 0 0 0];

end

    
