%% =================================================
% Function lab = getLabel(img, rob, len, res)
% --------------------------------------------------
% Creates a label, that is cspace mapping for that image, this function
% uses a hardcoded robot configuration which is:
%   1. A 2d bot with fixed origin at center of image
%   2. With 2 joints one at one at origin another at a length of 200 px
%   3. Length of 2nd arm is 200 px;
%   4. Width of all segments is 1 px;
%
% input:  img -> Is the source image
%         rob -> A 3-joint robot encoded as a SerialLink class
%         len -> Is an array of factor by which robot length has to be stretched
%         res -> Is the resolution of expected output [resxres]
% output: lab -> Is the label for the image
%
%%==================================================
function lab = getLabel(img, rob, len, res)
    %% Initialization
    lab = zeros(res, res);
    q   = linspace(0,2*pi,res);
    
    %% For each joint configuration check for collision
    for i = 1:res
        for j = 1:res
            lab(i,j) = isInColision(img, rob, len, [q(i) q(j)]);
        end
    end
end



