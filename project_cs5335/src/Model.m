classdef Model 
    properties
        %% Calculated properties
        rob;    % Holds the robot object
        res;    % Holds the current resolution of the world
        
        X;      % Holds the dataset QxR matrix of samples
        T;      % Holds the dataset QxU matrix of labels
        WorldLimits;    % Holds the world limits
    end;
    properties(Access=private)
        len;    % Is an array of factor by which robot length has to be stretched
    end;
    methods
        %% Setters
        function obj = set.X(obj,xIn)
        % Set X
            obj.X=xIn;
        end;       
        function obj = set.T(obj,tIn)
        % Set Target
            obj.T=tIn;
        end;
       
        %% Getters
        function X = get.X(obj)
        % Get X
            X = obj.X;
        end;      
        function T = get.T(obj)
        % Get Target
            T = obj.T;
        end;               
        function rob = get.rob(obj)
        % Get rob
            rob = obj.rob;
        end;        
        function res = get.res(obj)
        % Get resolution
            res = obj.res;
        end;        
        function res = get.WorldLimits(obj)
        % Gets WorldLimits
            res = obj.WorldLimits;
        end;
    end;
    
    methods(Static)
    end;
    
    methods(Access=public)
        function obj = Model(robIn, resIn, totLen)
        %% =================================================
        % Function obj = Model(obj, robIn, resIn, totLen)
        % --------------------------------------------------
        % Default constructor
        %
        % input:  robIn    -> Is the robot in the form of SerialLink
        %         resIn    -> Is the resolution of expected output [resxres]
        %         totLen -> Is the max length of the robot arm extension
        % output: obj -> Is the object of the model
        %===================================================
            obj.res = resIn;
            obj.rob = robIn;
            
            % Set scaling for the arm of robot
            scale = Constants.MAGNIFICATION * resIn / totLen;
            obj.len = zeros(length(robIn.links));
            for i=1:length(robIn.links)
                obj.len(i) = scale * robIn.links(i).a;
            end;
            
            obj.WorldLimits = (1/Constants.MAGNIFICATION) * [-1 1];
        end;
        
        
        function img = createImage(~, nObs, res)
        %% =================================================
        % Function img = createImage(nObs, res)
        % --------------------------------------------------
        % Reads an object from file and creates an image with that object inside.
        % Object is placed in one of the four quadrants of the target image.
        % Requires global variables
        %
        % input: nObs -> Number of obstacles to add
        %        res   -> Is the expected resolution of target image
        % output: img -> Is the image generated from the shapes
        %===================================================
            %% Initialization
            img = zeros(res);

            fnames  = dir(strcat(Constants.SOURCE_PATH, Constants.SOURCE_PTRN));
            numfids = length(fnames);

            nObs = randi(nObs);

            %% ADD Obstacles
            for i=1:nObs
                imgPath = strcat(Constants.SOURCE_PATH, fnames(randi(numfids)).name);
                obj  = imread(imgPath);

                qSize = floor(res/2);
                % If object occupies more than 1 quadrant resize image
                if(max(size(obj)) > qSize)  
                    obj = imresize(obj, [qSize qSize]);
                end
                [l, w] = size(obj);

                % Create a quadrant and place image into it
                quad = zeros(qSize);

                rS = randi(qSize-l+1); rE = rS + l -1;
                cS = randi(qSize-w+1); cE = cS + w -1;

                quad(rS:rE, cS:cE) = obj;

                % Place the quadrant in the final image
                dh = (randi(2)-1) * qSize +1;
                dv = (randi(2)-1) * qSize +1;

                %size
                img(dh:(dh+qSize-1), dv:(dv+qSize-1)) = quad;
                
                % Convert to binary image
                img = img>0;    
            end
        end;
        
        function lab = calculateLabel(obj, img, res)
        %% =================================================
        % Function lab = calculateLabel(img, res)
        % --------------------------------------------------
        % Creates a label, that is cspace mapping for that image, this function
        %
        % input:  img -> Is the source image
        %         res -> Is the resolution of expected output [resxres]
        % output: lab -> Is the label for the image
        % TODO: Multi dimensional output
        %===================================================
            %% Initialization
            lab = zeros(res, res);
            q   = linspace(0,2*pi,res);

            %% For each joint configuration check for collision
            for i = 1:res
                for j = 1:res
                    lab(i,j) = obj.isInColision(img, [q(i) q(j)]);
                end
            end
            % Convert to binary image
            lab = lab>0;  
        end;
        
        function out = addData(obj, xIn, tIn)
        %% =================================================
        % Function addData(obj, xIn, tIn)
        % --------------------------------------------------
        % Adds new row to the model
        %
        % input:  xIn -> Is the in data
        %         tIn -> Is the target or label for data
        % output: None.
        %===================================================
            obj.T = [obj.T tIn];
            obj.X = [obj.X xIn];
            out = obj;
        end;
    end;
    
    methods(Access=private)
        function col = isInColision(obj, img, q)
        %% =================================================
        % Function col = isInColision(img, len, q)
        % --------------------------------------------------
        % Checks if robot arm is in colision or not by expanding resolution. Total
        % length of robot arm is projected to 25% of resolution.
        %
        % input:  img -> Is the source image (mxn matrix of unit8)
        %         q   -> Array of joint angles to check
        % output: col -> True if any segment in arm is in colision
        %
        %===================================================
            %% Constants
            MAX_SEG = 10;

            %% Initialize
            imgSize = size(img);
            iL = imgSize(1)/2; iW = imgSize(2)/2;

            %% Calculate arm positions
            [~, temp] = obj.rob.fkine(q);
            pos(:,:) = temp(1:2, 4,:);
            % Add origin to the pos list
            pos = [0 0; pos'];      

            %% Sample points across arm of robot 
            l = size(pos,1);
            pts = zeros((l-1) * MAX_SEG, 1);
            s = 1; e = MAX_SEG;
            for i=1:(l-1)
                x = linspace(pos(i,1), pos(i+1,1), MAX_SEG);
                x = obj.len(i) * x' + iL;
                y = linspace(pos(i,2), pos(i+1,2), MAX_SEG); 
                y = obj.len(i) * y' + iW;

                x = round(x); y = round(y);
                % Append to point list
                pts(s:e) =  sub2ind(imgSize, x, y);
                s = e + 1; e = e + MAX_SEG;
            end
            pts = round(pts);

            %% Check if any point lies inside obstacle

            col = sum(img(pts)>0) > 0;
        end

    end;
end