
%% =================================================
% Function [rob, len] = create2DRobot(len)
% --------------------------------------------------
% Creates a SerialLink object with n joints
%
% input:  len -> Is 1xn matrix of length
% output: rob -> Is the robot of the given config
%         len -> Total length of robot
%%==================================================
function [rob, len] = create2DRobot(len)
    %% Create robot 
    L = zeros(length(len), 4);
    L(:,3) = len';
    
    rob  = SerialLink(L, 'name', 'S');
    
    %% Calculate meta parameters
    q0  = zeros(1, length(len));
    pos0 = rob.fkine(q0);
    pos0 = pos0(1:2,4);
    
    len = norm(pos0);
end