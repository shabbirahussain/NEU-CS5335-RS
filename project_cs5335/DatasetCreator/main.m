%% =================================================
% Function main()
% --------------------------------------------------
% Generates a dataset of source images
%%==================================================
function main()
    %% Constants
    createConstants();
    ARM_LEN1    = 0.5;      %px
    MAX_OBSTACLES = 2;
    MAGNIFICATION = 0.8;    % Size of robot relative to image
    
    DATASET_SIZE= 1;
    RESULT_DIM  = 101;
    
    %% Initialize
    iMag = 1/MAGNIFICATION;
    WorldLimits = iMag * [-1 1];
    
    %% Create a 2D Robot
    lenMat = [ARM_LEN1, ARM_LEN1];
    rob = create2DRobot(lenMat);
    
    % Recalibrate length to be <100% of visible area
    len = MAGNIFICATION * RESULT_DIM / sum(lenMat);
    lenMat = lenMat * len;
    
    %% Generate data samples
    for K = 1:DATASET_SIZE
        % Generate source image
        img  = createImage(MAX_OBSTACLES, RESULT_DIM);
        
        % Get label for image
        lab = getLabel(img, rob, lenMat, RESULT_DIM);
        
        % Save the results
        saveResults(img, lab);
    end
    
    % Plot results
    plotResults(img, lab, rob, WorldLimits);
    
    %% Export variables
    assignin('base', 'rob',rob);
    assignin('base', 'img',img);
    assignin('base', 'len',lenMat);
    assignin('base', 'lab',lab);
    
end


function plotResults(img, lab, rob, WorldLimits)
    %% Plot the C-Space
    figure(1);
    RI = imref2d(size(img)); 
    RI.XWorldLimits = WorldLimits; RI.YWorldLimits = WorldLimits;
    imshow(img,RI);

    figure(2);
    image(lab*255);
    
    % Show robot
    q1Init = [10 2];
    figure(1)
    rob.plot(q1Init,'jointdiam',0);
    
end


    