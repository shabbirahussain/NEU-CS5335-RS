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
    y2 = net(x2)  ; 
    [min(y2) max(y2)]
    y2 = reshape(y2,siz, siz);
    y2 = round(y2);
    t2 = T(:, num); t2 = reshape(t2,siz, siz);
    
    %% Plot results
    figure(4);imshow(t2);
    figure(3);imshow(y2);
end


