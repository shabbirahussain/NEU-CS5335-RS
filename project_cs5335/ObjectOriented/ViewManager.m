classdef ViewManager
    methods(Static)
        function showWorld(img, model)
        %% =================================================
        % Function showWorld(img, model)
        % --------------------------------------------------
        % Plots the world with the world with given parameters 
        %
        % input: img -> Is the dataset image to display
        %        model -> Is the model of the world
        %===================================================
            %% Initialize
            rob = model.rob;
            WorldLimits = model.WorldLimits;
            
            %% Plot the World
            f = ViewManager.getFigureHandle('WORLD'); 
            
            % Plot obstacle
            RI = imref2d(size(img)); 
            RI.XWorldLimits = WorldLimits;
            RI.YWorldLimits = WorldLimits;
            fig = imshow(img,RI, 'InitialMagnification', 'fit');
            set(fig,'AlphaData',img);

            % Show robot
            q1Init = [10 2];
            rob.plot(q1Init,'jointdiam',0);
            title('World');
        end;
        
        function showTarget(lab, time)
        %% =================================================
        % Function showTarget(lab)
        % --------------------------------------------------
        % Plots the Label Target
        %
        % input: lab -> Is the label image to show
        %        time -> Time taken for execution
        %===================================================
            f = ViewManager.getFigureHandle('TARGET'); 
            imshow(lab, 'InitialMagnification', 'fit');
            title(strcat('Target : ', num2str(time)));
        end;
        
        function showOutput(out, time)
        %% =================================================
        % Function showOutput(lab)
        % --------------------------------------------------
        % Plots the the output
        %
        % input: out  -> Is the output image to show
        %        time -> Time taken for execution
        %===================================================
            %% Plot the Output
            f = ViewManager.getFigureHandle('OUTPUT'); 
            imshow(out, 'InitialMagnification', 'fit');
            title(strcat('Neural Output : ', num2str(time)));
        end; 
        
        function showNet(net)
        %% =================================================
        % Function showNet(net)
        % --------------------------------------------------
        % Plots the network
        %
        % input: net -> Net to show
        %===================================================
            %% Plot the Output
            %f = ViewManager.getFigureHandle('NET'); 
            view(net);
            title('Network');
        end;
        
        function showHist(e)
        %% =================================================
        % Function showHist(e)
        % --------------------------------------------------
        % Plots the error histogram
        %
        % input: e -> Is the error array
        %===================================================
            %% Plot the Output
            %f = ViewManager.getFigureHandle('HIST'); 
            figure(1);
            ploterrhist(e);
            title('Histogram');
        end;
    end;
    
    methods(Access=private, Static)
        %%
        function f = getFigureHandle(plotType) 
        %% =================================================
        % Function f = getFigureHandle(plotType)
        % --------------------------------------------------
        % Returns a handle for the figure to plot
        %
        % input: plotTyp -> Is the type of plot to display
        % {'WORLD','TARGET', 'OUTPUT', 'NET', 'HIST'}
        % output: f -> Is the handle for the figure
        %===================================================
            f = findobj(0, 'Name', 'Dashboard');
            if isempty(f)
                f = figure(2);
                set(f, 'Name', 'Dashboard');
            end;
            switch(plotType)
%                 case 'NET'
%                     pos = [1, 2];
                case 'WORLD' 
                    pos = [1, 3];
                case 'TARGET'
                    pos = [2];
                case 'OUTPUT'
                    pos = [4];
%                 case 'HIST'
%                     pos = [7, 8];
            end;
            f = subplot(2,2, pos);
        end
   end
end
