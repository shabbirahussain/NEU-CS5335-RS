%% =================================================
% Function lab = getLabel(img, rob, len, res)
% --------------------------------------------------
% Creates a label, that is cspace mapping for that image, this function
%
% input:  img -> Is the source image
%         rob -> A 3-joint robot encoded as a SerialLink class
%         len -> Is an array of factor by which robot length has to be stretched
%         res -> Is the resolution of expected output [resxres]
% output: lab -> Is the label for the image
%
%===================================================
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



