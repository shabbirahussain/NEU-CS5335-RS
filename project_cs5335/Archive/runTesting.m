%% =================================================
% Function runTraining(res, num)
% --------------------------------------------------
% Loads a pretrained model and runs test on it.
%
% input: res -> Is the requested resolution
%        num -> (optional) is the testing intance number
%===================================================
function runTesting(res, num)
    objPath = Constants.OBJSTR_PATH;
    
    nmanName  = strcat('nman',num2str(res));
    
    if(~existin(nmanName, 'var'))
        modelPath = strcat(objPath, nmanName, '.mat');
        obj = load(modelPath);
        assignin('base', nmanName, obj.nman);
    end;
    nman = evalin('base',nmanName);
    if(nargin==2)
        nman.test(num);
    else
        nman.test;
    end;
end