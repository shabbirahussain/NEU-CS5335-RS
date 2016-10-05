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
function q = Q5(f1,f2,qInit,f1Target,f2Target)
    % Constants
    qDesired = [0, -0.7800, 0, 1.5700, 0, 3.1416, 0, -1.0000, 1.0000, 1.0000, -1.0000];
    alpha1 = 0.05;
    alpha2 = 0.05;
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
        
        % Calculate nullspace term
        siz = size(Ji);
        I   = eye(siz(1));
        NS = (I - Ji * J);
        
        % Find delta Q
        dQ = (qDesired - q);
        dQ = (alpha1 * Ji * dX) + (alpha2 * NS * dQ');
        q  = dQ' + q;
    end
end

function q = Q5_1(f1,f2,qInit,f1Target,f2Target)
    alpha1 = 0.05;
    alpha2 = 0.01;
    alpha3 = 0.065;
    
    R = [1 0 0; 0 1 0; 0 0 1];
    
    q = qInit;
    q1 = [q(:,1:7) q(:,8:9)];
    q2 = [q(:,1:7) q(:,10:11)];
    
    % Find arm target 
    [~, ALL1] = f1.fkine(q1);
    [~, ALL2] = f2.fkine(q2);
    armOffset = 0.5*(ALL1(:,:,9)+ALL2(:,:,9) - (ALL1(:,:,7)+ALL2(:,:,7)));
    armOffset = armOffset(1:3,4);
    armOffset(3) = 0;
    armTarget = 0.5*(f1Target + f2Target) - armOffset;
    
    pG  = [R armTarget; 0 0 0 1];
    scatter3(armTarget(1), armTarget(2), armTarget(3), '*');
    
    % Calculate arm position first
    q = [qInit(:,1:7) 0 0];
    for i=1:50
        [~,ALL] = f1.fkine(q);
        p1 = ALL(:,:,7);        % Calculate position at joint 7
        
        dX = tr2delta(p1, pG);
        
        J  = f1.jacob0(q);
        J(:,8:9) = 0;           % Disable finger1 joints
        
        dQ = alpha1 * pinv(J) * dX;
        q  = dQ' + q;
    end
    
    % Now calculate finger motion
    q1 = [q(:,1:7) qInit(:, 8: 9)];
    q2 = [q(:,1:7) qInit(:,10:11)];
    
    pG1 = [R f1Target; 0 0 0 1];
    pG2 = [R f2Target; 0 0 0 1];
    for i=1:50
        p1 = f1.fkine(q1);
        p2 = f2.fkine(q2);
        
        dX1 = tr2delta(p1, pG1);
        dX2 = tr2delta(p2, pG2);
        
        dQDot1 = (q1 - q);dQDot1(:,8:9)=0;
        dQDot2 = (q2 - q);dQDot2(:,8:9)=0;
        
        J1  = f1.jacob0(q1);
        J2  = f2.jacob0(q2);
        
        J1i = pinv(J1);
        J2i = pinv(J2);
        I   = eye(length(J1i));
        NS1 = (I - J1i * J1);
        NS2 = (I - J2i * J2);
        
        dQ1  = alpha2 * J1i * dX1 + alpha3 * NS1 * dQDot1';
        dQ2  = alpha2 * J2i * dX2 + alpha3 * NS2 * dQDot2';
        
        q1  = dQ1' + q1;
        q2  = dQ2' + q2;
        
    end
    q = [q1(:,1:9) q2(:,8:9)];
end
    
