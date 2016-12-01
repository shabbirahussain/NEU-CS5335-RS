function Stage1()
    BASE_PATH   = '/Users/shabbirhussain/Data/rs/';
    SOURCE_PATH = strcat(BASE_PATH, 'shapes/');
    SOURCE_PTRN = '*.gif';
    
    fnames  = dir(strcat(SOURCE_PATH, SOURCE_PTRN));
    numfids = length(fnames);

    for i=1:numfids
        imgPath = strcat(SOURCE_PATH, fnames(i).name);
        obj  = imfill(imread(imgPath))*255;
        imwrite(obj, imgPath, 'gif');
    end
end