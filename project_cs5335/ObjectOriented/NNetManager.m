classdef NNetManager
    properties
        worldModel;     % Stores the world model
        net;            % Stores an internal representation of net
    end;
    methods
        function obj = NNetManager(model, outputSize)
        %% =================================================
        % Function obj = NNetManager(model, outputSize)
        % --------------------------------------------------
        % Default constructor
        %
        % input:  model -> Is the model of the manager
        %         outputSize -> Is the required output layer size
        % output: Object of the NNetManager
        %%==================================================
            obj.worldModel = model;
            obj.net        = NNetManager.buildNet(outputSize);
        end;
        
        function out = train(obj, x, t)
        %% =================================================
        % Function obj = train(x, t)
        % --------------------------------------------------
        % Trains the network with the given parameters
        %
        % input:  x -> Is the input,  RxQ matrix of inputs
        %         t -> Is the target, UxQ matrix of outputs
        % output: out -> Object of the NNetManager
        %%==================================================
            %% Train the Network
            [obj.net, ~] = train(obj.net,x,t);

            %% Test the Network
            y = obj.net(x);
            e = gsubtract(t,y);
            %     performance = perform(net,t,y);

            %% View the Network
            ViewManager.showNet(obj.net);
            
            %% Show plots
            ViewManager.showHist(e);

            %% Test Network 
            obj.testNet();
            
            out = obj;
        end;
        
        function testNet(obj, num)
        %% =================================================
        % Function testNet(obj)
        % --------------------------------------------------
        % Plos a sample out of given X by calculating results from network.
        %
        % input:  num -> A sample number to plot
        % output: None.
        %%==================================================
            %% Initialize
            X = obj.worldModel.X;
            if (nargin==1)
                num = randi(size(X,2));
            end;

            %% Calculate results
            x2  = X(:,num) ; iSize = sqrt(length(x2));
            img = reshape(x2, iSize, iSize);
            
            %% Time net and execute
            tic;    
                y2 = obj.net(x2); 
                y2 = mapminmax(y2'); y2 = y2'>0;
            tNet = toc;
            oSize = sqrt(length(y2));
            y2 = reshape(y2, oSize, oSize);
            
            %% Time the normal calculation
            tic     
                t2 = obj.worldModel.calculateLabel(img, oSize);
            tNor = toc;
            
            
            %% Plot results
            ViewManager.showWorld(img, obj.worldModel);
            ViewManager.showTarget(t2, tNor); 
            ViewManager.showOutput(y2, tNet);
        end;
    end;
    methods(Access=private, Static)
        function net = buildNet(outputSize)
        %% =================================================
        % Function [net, tr] = buildNet(outputSize)
        % --------------------------------------------------
        % Builds a network 
        % input: outputSize -> Is the size of output layer
        % output: net -> Network object
        %%==================================================
            %% Constants
            numNeuronPerLayer = 100;
            numInputs     = 1;                         % Number of inputs
            numLayers     = 1;                         % Total number of layers - input
            biasConnect   = ones(numLayers, 1);        % Bias connect to all layers 
            inputConnect  = [ones(1, numInputs); zeros(numLayers-1, numInputs)];
            layerConnect  = [zeros(1, numLayers); eye(numLayers-1, numLayers)]; % Feed forward network
            outputConnect = [zeros(1, numLayers-1) 1];


            %% Choose a Training Function
            % For a list of all training functions type: help nntrain
            % 'trainlm' is usually fastest.
            % 'trainbr' takes longer but may be better for challenging problems.
            % 'trainscg' uses less memory. Suitable in low memory situations.
            trainFcn = 'trainrp';%'trainscg'; 


            %% Create network
            net = network(numInputs,numLayers,biasConnect,inputConnect,layerConnect, outputConnect);
            
            net.trainFcn = trainFcn; % Scaled conjugate gradient backpropagation.
            net.adaptFcn = 'adaptwb';
            net.trainParam.min_grad = 10.e-50;
            net.trainParam.epochs   = 1000;


            %net.plotFcns   = {'plotfit'};

            % Setup Division of Data for Training, Validation, Testing
            % net.divideParam.trainRatio = 70/100;
            % net.divideParam.valRatio = 15/100;
            % net.divideParam.testRatio = 15/100;

            %% Set layer parameters
            for i=1:numLayers-1
                net.layers{i}.transferFcn = 'purelin';       %'tansig''logsig';
                net.layers{i}.size  = numNeuronPerLayer;    % number of neurons
                net.layers{i}.initFcn = 'initwb';
            end

            net.layers{numLayers}.transferFcn = 'purelin';  %'purelin';%'hardlim';
            net.layers{numLayers}.size  = outputSize;  
            net.outputs{numLayers}.processFcns = {'mapminmax'};
            net.outputs{numLayers}.processParams{1} = struct('ymin',0,'ymax',1);
        end;
    end;
end

