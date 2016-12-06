%% =================================================
% Function loadModel(res)
% --------------------------------------------------
% Loads a pretrained model 
%
% input: res -> Is the requested resolution
%===================================================
function loadModel(res)
    objPath = Constants.OBJSTR_PATH;
    
    nmanName  = strcat('nman',num2str(res));
    
    if(~exist(nmanName, 'var'))
        modelPath = strcat(objPath, nmanName, '.mat');
        obj = load(modelPath);
        assignin('base', nmanName, obj.nman);
    end;
end