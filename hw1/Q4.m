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
    % Constants
    alpha = 0.05;
    R    = [1 0 0; 0 1 0; 0 0 1];
    FIL  = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];
    
    % Find goal positions
    pGf1 = [R  f1Target; 0 0 0 1];
    pGf2 = [R  f2Target; 0 0 0 1];
    
    q = qInit;
    for i=1:50
        % Calculate current angles
        qf1 = [q(:, 1:7) q(:,  8: 9)];  % F1 angles
        qf2 = [q(:, 1:7) q(:, 10:11)];  % F2 angles
        
        % Calculate current positions 
        pf1 = f1.fkine(qf1);
        pf2 = f2.fkine(qf2);
        
        % Calculate current delta 
        dXf1 = tr2delta(pf1, pGf1);     % 6x1
        dXf2 = tr2delta(pf2, pGf2);     % 6x1
        dX   = [dXf1; dXf2];            % 12x1
        
        % Calculate Jacobian
        Jf1  = f1.jacob0(qf1);              % 6x9
        Jf1  = [Jf1(:,1:7) Jf1(:,8:9) FIL]; % 6x11 
        Jf2  = f2.jacob0(qf2);              % 6x9
        Jf2  = [Jf2(:,1:7) FIL Jf2(:,8:9)]; % 6x11 
        J    = [Jf1; Jf2];                  % 12x11
        
        % Calculate Jacobian inverse
        Ji   = pinv(J);                     % 11x12
        
        % Find delta Q
        dQ = alpha * Ji * dX;
        q  = dQ' + q;
    end
end