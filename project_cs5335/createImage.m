
%% =================================================
% Function img = createImage(nObs, res)
% --------------------------------------------------
% Reads an object from file and creates an image with that object inside.
% Object is placed in one of the four quadrants of the target image.
% Requires global variables
%
% input: nObs -> Number of obstacles to add
%        res   -> Is the expected resolution of target image
% output: img -> Is the image generated from the shapes
%%==================================================
function img = createImage(nObs, res)
    %% Constants
    global SOURCE_PATH SOURCE_PTRN;
    
    %% Initialization
    img = zeros(res);
    
    fnames  = dir(strcat(SOURCE_PATH, SOURCE_PTRN));
    numfids = length(fnames);
    
    nObs = randi(nObs);
    
    %% ADD Obstacles
    for i=1:nObs
        imgPath = strcat(SOURCE_PATH, fnames(randi(numfids)).name);
        %imgPath = strcat(SOURCE_PATH, 'brick-20.gif');
        obj  = imread(imgPath);

        qSize = floor(res/2);
        % If object occupies more than 1 quadrant resize image
        if(max(size(obj)) > qSize)  
            obj = imresize(obj, [qSize qSize]);
        end
        [l, w] = size(obj);

        % Create a quadrant and place image into it
        quad = zeros(qSize);

        rS = randi(qSize-l+1); rE = rS + l -1;
        cS = randi(qSize-w+1); cE = cS + w -1;

        quad(rS:rE, cS:cE) = obj;

        % Place the quadrant in the final image
        dh = (randi(2)-1) * qSize +1;
        dv = (randi(2)-1) * qSize +1;

        %size
        img(dh:(dh+qSize-1), dv:(dv+qSize-1)) = quad;
    end
end