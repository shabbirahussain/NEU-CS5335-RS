%% =================================================
% Function testNet(net, X, T, num)
% --------------------------------------------------
% Plos a sample out of given X by calculating results from network.
%
% input:  net -> Is instance of network
%         X   -> Is a RxQ matrix of features
%         Y   -> Is a UxQ matrix of labels
%         num -> A sample number to plot
% output: None.
%%==================================================
function testNet(net, X, T, num)
    %% Initialize
    if (nargin==3)
        num = randi(size(X,2));
    end;
    
    %% Calculate results
    x2 = X(:,num) ; siz = sqrt(length(x2));
    
    y2 = net(x2)  ; y2 = mapminmax(y2'); y2 = y2'>0;
    y2 = reshape(y2,siz, siz);
    
    t2 = T(:, num); t2 = reshape(t2,siz, siz);
    
    %% Plot results
    figure(1);
    subplot(1,2,1);imshow(t2); title('Target');
    subplot(1,2,2);imshow(y2); title('Neural Output');
end

