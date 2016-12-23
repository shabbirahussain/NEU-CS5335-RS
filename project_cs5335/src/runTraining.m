%% =================================================
% Function runTraining(res)
% --------------------------------------------------
% Runs the training for the specified resolution 
%
% input: res -> Is the requested resolution
%===================================================
function runTraining(res)
    objPath = Constants.OBJSTR_PATH;

    dsName = strcat('2DModel_', num2str(res), 'x', num2str(res), '.mat');
    model = DatasetManager.loadDataset(dsName, res, 1000);
    
    nman = NNetManager(model);
    nman = nman.train;
    nman.test;
    
    nmanName = strcat('nman', num2str(res));
    save(strcat(objPath, nmanName, '.mat'), 'nman', '-v7.3');
    assignin('base', nmanName, nman);
end
