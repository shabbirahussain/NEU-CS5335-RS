%% =================================================
% Function data_generator()
% --------------------------------------------------
% Generates a dataset of source images
%%==================================================
function data_generator()
    %% Constants
    createConstants();
    ARM_LEN1    = 0.5;  %px
    MAX_OBSTACLES = 1;
    
    DATASET_SIZE= 1;
    RESULT_DIM  = 101;
    
    %% Initialize
    
    
    %% Create a 2D Robot
    q1Init = [10 2];
    lenMat = [ARM_LEN1, ARM_LEN1];
    rob = create2DRobot(lenMat);
    
    % Recalibrate length to be 50% of visible area
    len = 0.5 * RESULT_DIM / sum(lenMat);
    lenMat = lenMat * len;
    
    %% Generate data samples
    for K = 1:DATASET_SIZE
        % Generate source image
        img  = createImage(MAX_OBSTACLES, RESULT_DIM);
        RI = imref2d(size(img)); RI.XWorldLimits = [-1 1]; RI.YWorldLimits = [-1 1];
        imshow(img,RI);
        
        % Get label for image
        lab = getLabel(img, rob, lenMat, RESULT_DIM);
    end
    rob.plot(q1Init,'jointdiam',0);
    figure(2);
    image(lab*255);
    assignin('base', 'rob',rob);
end





    