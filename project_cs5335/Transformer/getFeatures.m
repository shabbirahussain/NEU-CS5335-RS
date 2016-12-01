%% =================================================
% Function [X, Y] = getFeatures(name, size)
% --------------------------------------------------
% Reads a feature file if present by name if not then scans the input to build one feature file.
%
% input:  name -> Is the name of feature file to load
%         size -> Is the dimension of the feature image
% output: X    -> Is an matrix of features
%         Y    -> Is an matrix of labels
%%==================================================
function [X, Y] = getFeatures(name, size)
    %% Constants
    createConstants();
    global OBJSTR_PATH;
    
    %% Load dataset
    fullFileName = strcat(OBJSTR_PATH, name);
    if exist(fullFileName, 'file')
        % Load the dataset
        obj = load(fullFileName);
        X = obj.obj.X; Y = obj.obj.Y;
    else
        % Parse and create new matrix
        [X, Y] = parseFeatures(size);
        % Save the dataset for future use
        obj = struct('X', X, 'Y', Y);
        save(fullFileName, 'obj', '-v7.3');
    end
    
end


%% =================================================
% Function [X, Y] = parseFeatures(size)
% --------------------------------------------------
% Reads features to form an array
%
% input:  size -> Is the dimension of the feature image
% output: X    -> Is an matrix of features
%         Y    -> Is an matrix of labels
%%==================================================
function [X, Y] = parseFeatures(size)
    global TARGET_PATH LABEL_PATH;
    
    fnames  = dir(strcat(TARGET_PATH, '*.png'));
    numfids = length(fnames);
    
    X = zeros(numfids, size * size);
    Y = zeros(numfids, size * size);
    for i=1:numfids
        X(i,:) = readFile(TARGET_PATH, fnames(i).name);
        X(i,:) = readFile(LABEL_PATH,  fnames(i).name);
    end
end

%% =================================================
% Function arr = readFile(tgtPath, name)
% --------------------------------------------------
% Reads the png image to a vector
%
% input: tgtPath -> Is the target path of the folder
%        name    -> Is the name of the file to read
% output: arr    -> Returns the output in an array
%%==================================================
function arr = readFile(tgtPath, name)
    imgPath = strcat(tgtPath, name);
    obj     = imread(imgPath);
    arr     = reshape(obj, 1, numel(obj));
end