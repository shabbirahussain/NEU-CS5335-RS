model10 = DatasetManager.loadDataset('2DModel_10x10.mat', 10, 2000);
nman10 = NNetManager(model10);
nman10 = nman10.train;
nman10.testNet;

model100 = DatasetManager.loadDataset('2DModel_100x100.mat', 100, 2000);
nman100 = NNetManager(model100);
nman100 = nman100.train;
nman100.testNet;