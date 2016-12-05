classdef DatasetManager 
    methods(Access=public, Static)
        function model = createNewDataset(res, count)
        %% =================================================
        % Function model = createNewDataset(res, count)
        % --------------------------------------------------
        % Generates a dataset of source images
        % input:  res -> Is the result dimension of the output
        %         count -> Is the number of instances to generate
        % outpur: model -> Is the model generated from the configuration
        %===================================================
            model  = DatasetManager.buildModel(res);
            % Generate data samples
            for K = 1:count
                % Generate source image
                img = model.createImage(Constants.MAX_OBSTACLES, res);

                % Get label for image
                lab = model.calculateLabel(img, res);

                % Save the results
                DatasetManager.saveResults(img, lab);
                img = reshape(img, numel(img), 1);
                lab = reshape(lab, numel(img), 1);
                model = model.addData(img, lab);
            end
            
            name = strcat(Constants.OBJSTR_PATH, 'model_', datestr(now,'mmddyyHHMMSSFFF'), '.mat');
            save(name, 'model', '-v7.3');
        end;
        
        function model = loadDataset(name, res, count)
        %% =================================================
        % Function model = loadDataset(name, res, count)
        % --------------------------------------------------
        % Loads a dataset or builds a new one
        % input:  name -> Is the name of the file to load
        %         res  -> Is the resolution of the model to load
        %          count -> (optional) number of records to load
        % outpur: model -> Is the model generated from the configuration
        %===================================================
            name  = strcat(Constants.OBJSTR_PATH, name);
            
            if exist(name, 'file')
                % Load the dataset
                obj   = load(name);
                model = obj.model;
            else
                % Parse and create new matrix
                [X, T] = DatasetManager.parseFeatures(res);
                model  = DatasetManager.buildModel(res);
                model.X = X; model.T = T;
                save(name, 'model', '-v7.3');
            end;
            
            if(nargin==3)
                %model.X = datasample(model.X, count, 2);
                %model.T = datasample(model.T, count, 2);
                
                model.X = model.X(:, 1:count);
                model.T = model.T(:, 1:count);
            end;
        end;
    end;
    
    methods(Access=private, Static)
        function [rob, len] = create2DRobot()
        %% =================================================
        % Function rob = create2DRobot(len)
        % --------------------------------------------------
        % Creates a SerialLink object with n joints
        %
        % input:  len -> Is 1xn matrix of length
        % output: rob -> Is the robot of the given config
        %===================================================
            %% Create robot 
            lenMat = [0.5, 0.5];
            
            L = zeros(length(lenMat), 4);
            L(:,3) = lenMat';                   % Set arm length

            rob  = SerialLink(L, 'name', 'S');  % Create a robot
            len  = sum(lenMat);
        end
        
        function saveResults(img, lab)
        %% =================================================
        % Function saveResults(img, lab)
        % --------------------------------------------------
        % Saves the results to the file
        %
        % input: img -> Is the dataset image to save
        %        lab -> Is the label image
        %===================================================
            %% Constants
            name = strcat(datestr(now,'mmddyyHHMMSSFFF'), '.', Constants.OUTPUT_FORMAT);
            
            tPath = strcat(Constants.TARGET_PATH, name);
            lPath = strcat(Constants.LABEL_PATH , name);

            img = img > 0; % Convert to boolean

            imwrite(img, tPath, Constants.OUTPUT_FORMAT);
            imwrite(lab, lPath, Constants.OUTPUT_FORMAT);
        end;
        
        function [X, Y] = parseFeatures(size)
        %% =================================================
        % Function [X, Y] = parseFeatures(size)
        % --------------------------------------------------
        % Reads features to form an array
        %
        % input:  size -> Is the dimension of the feature image
        % output: X    -> Is an matrix of features
        %         Y    -> Is an matrix of labels
        %%==================================================
            fnames  = dir(strcat(Constants.TARGET_PATH, '*.png'));
            numfids = length(fnames);

            X = zeros(size * size, numfids);
            Y = zeros(size * size, numfids);
            for i=1:numfids
                X(:,i) = DatasetManager.readFile(Constants.TARGET_PATH, fnames(i).name, size);
                Y(:,i) = DatasetManager.readFile(Constants.LABEL_PATH,  fnames(i).name, size);
            end
        end;
        function arr = readFile(tgtPath, name, size)
        %% =================================================
        % Function arr = readFile(tgtPath, name)
        % --------------------------------------------------
        % Reads the png image to a vector
        %
        % input: tgtPath -> Is the target path of the folder
        %        name    -> Is the name of the file to read
        %        size    -> Is the dimension of the feature image
        % output: arr    -> Returns the output in an array
        %%==================================================
            imgPath = strcat(tgtPath, name);
            obj     = imread(imgPath);
            obj     = imresize(obj, [size, size]);
            arr     = reshape(obj, numel(obj), 1);
        end;
        
        function model = buildModel(res)
        %% =================================================
        % Function arr = readFile(tgtPath, name)
        % --------------------------------------------------
        % Reads the png image to a vector
        %
        % input: res    -> Is the required resolution
        % output: model -> Returns a new model built from parameters
        %%==================================================
            %% Create a 2D Robot
            [rob, totLen] = DatasetManager.create2DRobot();
            
            %% Generate new model
            model = Model(rob, res, totLen);    % Create new model
        end;
   end
end
