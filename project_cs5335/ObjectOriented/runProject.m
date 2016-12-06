function runProject(res)
    objPath = '/Users/shabbirhussain/Data/rs/objectStore/';

    dsName = strcat('2DModel_', num2str(res), 'x', num2str(res), '.mat');
    model = DatasetManager.loadDataset(dsName, res, 1000);
    
    nman = NNetManager(model);
    nman = nman.train;
    nman.test;
    
    nmanName = strcat('nman', num2str(res));
    save(strcat(objPath, nmanName, '.mat'), 'nman', '-v7.3');
    assignin('base', nmanName, nman);
end




% objPath = '/Users/shabbirhussain/Data/rs/objectStore/';
% 
% model10 = DatasetManager.loadDataset('2DModel_10x10.mat', 10, 2000);
% nman10 = NNetManager(model10);
% clear model10;
% nman10 = nman10.train;
% nman10.testNet;
% save(strcat(objPath,'nman', num2str(size), '.mat'), 'nman', '-v7.3'); 
% 
% model100 = DatasetManager.loadDataset('2DModel_100x100.mat', 100, 2000);
% nman100 = NNetManager(model100);
% clear model100;
% nman100 = nman100.train;
% nman100.test; 
% save