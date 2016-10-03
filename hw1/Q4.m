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
    alpha1 = 0.05;
    alpha2 = 0.01;
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
        
        J1  = f1.jacob0(q1);
        %J1(:,1:7) = 0;          % Disable arm joints
        J1(:,1) = 0;
        J1(:,3) = 0;
        J1(:,7) = 0;
        J2  = f2.jacob0(q2);
        %J2(:,1:7) = 0;          % Disable arm joints
        J2(:,1) = 0;
        J2(:,3) = 0;
        J2(:,7) = 0;
        
        dQ1  = alpha2 *pinv(J1) * dX1;
        dQ2  = alpha2 *pinv(J2) * dX2;
        
        q1  = dQ1' + q1;
        q2  = dQ2' + q2;
        
    end
    %q = [q(:,1:7) q1(:,8:9) q2(:,8:9)];
    q = [q1(:,1:9) q2(:,8:9)];
end

function q = Q4_1(f1,f2,qInit,f1Target,f2Target)
    alpha = 0.05;
    
    q = qInit;
    q1 = [q(:,1:7) q(:,8:9)];
    q2 = [q(:,1:7) q(:,10:11)];
    
    
    R = [1 0 0; 0 1 0; 0 0 1];
    pG1 = [R f1Target; 0 0 0 1];
    pG2 = [R f2Target; 0 0 0 1];
    
    for i=1:50
        p1 = f1.fkine(q1);
        p2 = f2.fkine(q2);
        
        dX1 = tr2delta(p1, pG1);
        dX2 = tr2delta(p2, pG2);
        
        J1  = f1.jacob0(q1);
        J1(:,1) = 0;
        J1(:,3) = 0;
        J1(:,7) = 0;
        J2  = f2.jacob0(q2);
        J2(:,1) = 0;
        J2(:,3) = 0;
        J2(:,7) = 0;
        
        dQ1  = alpha * pinv(J1) * dX1;
        dQ2  = alpha * pinv(J2) * dX2;
        
        q1  = dQ1' + q1;
        q2  = dQ2' + q2;
        
    end
    q = q1;
    q = [q(:,1:7) q1(:,8:9) q2(:,8:9)];
end