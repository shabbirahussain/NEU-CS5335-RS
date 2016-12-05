%% =================================================
% Function saveResults(img, lab)
% --------------------------------------------------
% Saves the results to the file
%
% input: img -> Is the dataset image to save
%        lab -> Is the label image
%===================================================
function saveResults(img, lab)
    %% Constants
    global TARGET_PATH LABEL_PATH OUTPUT_FORMAT;
    name = strcat(datestr(now,'mmddyyHHMMSSFFF'), '.',OUTPUT_FORMAT);
    %name = 'temp.png';
    
    tPath = strcat(TARGET_PATH, name);
    lPath = strcat(LABEL_PATH , name);
    
    img = img > 0; % Convert to boolean
    
    imwrite(img, tPath, OUTPUT_FORMAT);
    imwrite(lab, lPath, OUTPUT_FORMAT);
end