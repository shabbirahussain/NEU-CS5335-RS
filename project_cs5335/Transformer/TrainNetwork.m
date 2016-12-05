%% =================================================
% Function [net, tr] = trainNetwork1(x, t)
% --------------------------------------------------
% Builds a network and trains it with parameters given
%
% input:  x -> Is a RxQ features matrix
%         t -> Is a UxQ target matrix
% output: X    -> Is an matrix of features
%         Y    -> Is an matrix of labels
%%==================================================
function [net, tr] = trainNetwork(x, t)
    
    % network(numInputs,numLayers,biasConnect,inputConnect,layerConnect, outputConnect)
    % takes additional optional arguments and returns a neural
    %    network with the following properties defined:
    %      numInputs     - Number of inputs, 0.
    %      numLayers     - Number of layers, 0.
    %      biasConnect   - numLayers-by-1 Boolean vector, zeros.
    %      inputConnect  - numLayers-by-numInputs Boolean matrix, zeros.
    %      layerConnect  - numLayers-by-numLayers Boolean matrix, zeros.
    %      outputConnect - 1-by-numLayers Boolean vector, zeros.

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
     
    %% Initialize
    outputSize    = size(t,1);
   
    %net.plotFcns   = {'plotfit'};
    
    % Setup Division of Data for Training, Validation, Testing
%     net.divideParam.trainRatio = 70/100;
%     net.divideParam.valRatio = 15/100;
%     net.divideParam.testRatio = 15/100;
    
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
    
    %% Train the Network
    [net,tr] = train(net,x,t);

    %% Test the Network
%     y = net(x);
%     e = gsubtract(t,y);
%     performance = perform(net,t,y);

    %% View the Network
    %view(net)
    
    %% Test Network 
    testNet(net, x, t);

    %% Plots
    % Uncomment these lines to enable various plots.
    %figure, plotperform(tr)
    %figure, plottrainstate(tr)
    %figure, ploterrhist(e)
    %figure, plotregression(t,y)
    %figure, plotfit(net,x,t)
end