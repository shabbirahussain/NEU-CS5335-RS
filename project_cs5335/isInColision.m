%% =================================================
% Function col = isInColision(img, rob, len, q)
% --------------------------------------------------
% Checks if robot arm is in colision or not by expanding resolution. Total
% length of robot arm is projected to 25% of resolution.
%
% input:  img -> Is the source image (mxn matrix of unit8)
%         rob -> A 3-joint robot encoded as a SerialLink class
%         len -> Is an array of factor by which robot length has to be stretched
%         q   -> Array of joint angles to check
% output: col -> True if any segment in arm is in colision
%
%%==================================================
function col = isInColision(img, rob, len, q)
    %% Constants
    MAX_SEG = 20;
    
    %% Initialize
    [iL, iW] = size(img);
    
    %% Calculate arm positions
    [~, temp] = rob.fkine(q);
    pos(:,:) = temp(1:2, 4,:);
    % Add origin to the pos list
    pos = [0 0; pos'];      
    
    %% Sample points across arm of robot 
    l = size(pos,1);
    pts = zeros((l-1) * MAX_SEG, 1);
    s = 1; e = MAX_SEG;
    for i=1:(l-1)
        x = linspace(pos(i,1), pos(i+1,1), MAX_SEG); x=len(i) * x' + iL;
        y = linspace(pos(i,2), pos(i+1,2), MAX_SEG); y=len(i) * y' + iW;
        
        % Append to point list
        pts(s:e) =  y + (i-1)*x;
        s = e + 1; e = e + MAX_SEG;
    end
    pts = round(pts);
    
    %% Check if any point lies inside obstacle
    
    col = sum(img(pts)>0) > 0;
end
