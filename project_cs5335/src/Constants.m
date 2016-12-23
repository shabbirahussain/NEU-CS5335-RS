classdef Constants 
    properties(Constant, Access=private)
        %% Constants
        BASE_PATH   = '/Users/shabbirhussain/Data/rs/';
    end;
    properties(Constant, Access=public)
        %% Constants
        SOURCE_PATH = strcat(Constants.BASE_PATH, 'shapes/');
        TARGET_PATH = strcat(Constants.BASE_PATH, 'dataset/');
        LABEL_PATH  = strcat(Constants.BASE_PATH, 'labels/');
        OBJSTR_PATH = strcat(Constants.BASE_PATH, 'objectStore/');

        SOURCE_PTRN = '*.gif';
        OUTPUT_FORMAT = 'png';
        MAX_OBSTACLES = 2;
        
        MAGNIFICATION = 0.8;    % Size of robot relative to image
    end;
end