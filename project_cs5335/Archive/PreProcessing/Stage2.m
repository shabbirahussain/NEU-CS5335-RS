%% Make all images binary
function Stage2(folder)
    BASE_PATH   = '/Users/shabbirhussain/Data/rs/';
    SOURCE_PATH = strcat(BASE_PATH, folder, '/');
    SOURCE_PTRN = '*.png';
    
    fnames  = dir(strcat(SOURCE_PATH, SOURCE_PTRN));
    numfids = length(fnames);

    for i=1:numfids
        imgPath = strcat(SOURCE_PATH, fnames(i).name);
        obj     = imread(imgPath)>0;
        imwrite(obj, imgPath, 'png');
    end
end