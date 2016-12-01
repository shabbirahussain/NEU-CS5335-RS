%% =================================================
% Function createConstants()
% --------------------------------------------------
% Sets the environment variables for the program
%%==================================================
function createConstants()
    global SOURCE_PATH TARGET_PATH LABEL_PATH SOURCE_PTRN;
    
    BASE_PATH   = '/Users/shabbirhussain/Data/rs/';
    SOURCE_PATH = strcat(BASE_PATH, 'shapes/');
    TARGET_PATH = strcat(BASE_PATH, 'dataset/');
    LABEL_PATH  = strcat(BASE_PATH, 'labels/');
    SOURCE_PTRN = '*.gif';
end
