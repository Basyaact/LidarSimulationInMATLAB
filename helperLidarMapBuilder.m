%helperLidarMapBuilder Build a map from Lidar
%
%   This is an example helper class that is subject to change or removal in
%   future releases.
%
%   helperLidarMapBuilder implements a map building algorithm from Lidar
%   data by registering successive point clouds from a Lidar.
%
%   mapBuilder = helperLidarMapBuilder() creates a helperLidarMapBuilder
%   object to build a map using Lidar data.
%
%   mapBuilder = helperLidarMapBuilder(Name,Value) specifies additional
%   name-value pair arguments as described below:
%
%   'DownsamplePercent'             Percentage of random sampling used to
%                                   downsample point cloud data specified
%                                   as a value between 0-1. Increasing this
%                                   value retains more points while
%                                   improving registration at the cost of
%                                   computation time.
%
%                                   Default: 0.25
%
%   'RegistrationGridStep'          Grid step used to voxelize the incoming
%                                   point cloud during point cloud
%                                   registration.
%
%                                   Default: 3
%
%   'MergeGridStep'                 Grid step used to merge the transformed
%                                   point cloud after registration.
%
%                                   Default: 0.5
%
%   'Verbose'                       Flag specifying whether or not to
%                                   display progress information.
%
%                                   Default: false
%
%   helperLidarMapBuilder properties:
%   DownsamplePercent       - Percentage of random sampling
%   RegistrationGridStep    - Registration grid step to voxelize fixed pointCloud
%   MergeGridStep           - Grid step to voxelize while merging
%   Map                     - Point cloud map
%   ViewSet                 - Point cloud view set
%
%   helperLidarMapBuilder methods:
%   configureLoopDetector   - Configure loop closure detection
%   updateMap               - Update map with new pointCloud
%   updateDisplay           - Update map display
%   optimizeMapPoses        - Optimize map poses using pose graph optimization
%   rebuildMap              - Rebuild point cloud map from view set
%   processPointCloud       - Process pointCloud to remove ground and ego vehicle
%   reset                   - Reset map builder
%
%   Example : Build a map from Velodyne Lidar
%   -----------------------------------------
%   % Create a velodyne file reader object
%   veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');
%
%   % Create a map builder object
%   mapBuilder = helperLidarMapBuilder('RegistrationGridStep', 4.5);
%   
%   skipFrames = 5;
%   numFrames  = 200;
%
%   closeDisplay = false;
%   for n = 1 : skipFrames : numFrames-skipFrames
%       % Read lidar frame
%       ptCloud = readFrame(veloReader, n);
%
%       % Update map builder
%       updateMap(mapBuilder, ptCloud);
%
%       % Update map display
%       updateDisplay(mapBuilder, closeDisplay);
%   end
%
%   % Display accumulated map
%   figure, pcshow(mapBuilder.Map)
%
%   See also pcregisterndt, pcdownsample, pcmerge, rigid3d, affine3d.

% Copyright 2019 MathWorks, Inc.

classdef helperLidarMapBuilder < handle
    
    properties (Access = public)
        %DownsamplePercent
        %   Percentage of random sampling used to downsample point cloud
        %   data specified as a value between 0-1. Increasing this value
        %   retains more points while improving registration at the cost of
        %   computation time.
        %
        %   Default: 0.25
        DownsamplePercent(1,1)  double
        
        %RegistrationGridStep
        %   Grid step used to voxelize the incoming point cloud during
        %   point cloud registration.
        %
        %   Default: 3
        RegistrationGridStep(1,1) double
        
        %MergeGridStep
        %   Grid step used to merge the transformed point cloud after
        %   registration.
        %
        %   Default: 0.5
        MergeGridStep(1,1) double
    end
    
    properties (SetAccess = protected)
        %Map
        %   Map of built point cloud. The map is built relative to the
        %   first incoming point cloud.
        Map(1,1) pointCloud = pointCloud(zeros(0,0,3,'single'));
        
        %AccumTform
        %   Accumulated transformation, specified as a rigid3d object.
        AccumTform(1,1) rigid3d
        
        %RelativePositions
        %   Relative positions of the sequence of Lidar frames, specified
        %   as an M-by-3 matrix of [x, y, z] positions relative to the
        %   first Lidar frame.
        RelativePositions = zeros(0, 3);
        
        %Axes
        %   Handle to axes showing graphics display
        Axes
        
        %ViewSet
        %   A pcviewset object. The pcviewset object manages point cloud
        %   odometry data as a set of views and connections.
        ViewSet(1,1) pcviewset
        
        %LoopDetector
        %   A helperLoopClosureDetector object. The
        %   helperLoopClosureDetector object detects loop closures during
        %   map building.
        LoopDetector
    end
    
    properties (Access = protected)
        %Fixed
        %   Fixed point cloud
        Fixed(1,1) pointCloud = pointCloud(zeros(0,0,3,'single'));
        
        %Moving
        %   Moving point cloud
        Moving(1,1) pointCloud = pointCloud(zeros(0,0,3,'single'));
        
        %PreviousTform
        %   Previous transformation, specified as an rigid3d object.
        PreviousTform(1,1) rigid3d
        
        %IsInitialized
        %   Flag indicating whether or not map building has been
        %   initialized.
        IsInitialized(1,1) logical = false;
        
        %GroundRemovalParameters
        %   Parameters used for ground removal, specified as a struct with
        %   fields for MaxDistance and MaxAngularDistance
        GroundRemovalParameters = struct(...
            'MaxDistance',          0.4, ...
            'MaxAngularDistance',   5);
        
        %LoopConfirmationRMSE
        %   RMSE below which a loop closure candidate is confirmed. If the
        %   RMSE obtained through registration is above this value, the
        %   loop closure candidate is rejected.
        LoopConfirmationRMSE(1,1) double = 4.5;
        
        %ScatterPoints
        %   Handle to scatter object for drawing map points
        ScatterPoints
        
        %ScatterTrajectory
        %   Handle to scatter object for drawing trajectory
        ScatterTrajectory
        
        %ViewId
        %   Current view Id
        ViewId
        
        %Verbose
        %   Flag indicating whether or not progress information is
        %   displayed.
        Verbose
        
        %DetectLoops
        %   Flag to indicate whether or not loop closures are detected.
        %
        %   Default: false
        DetectLoops(1,1) logical
    end
    
    methods
        %------------------------------------------------------------------
        function this = helperLidarMapBuilder(varargin)
            
            params = parseInputs(varargin{:});
            
            this.DownsamplePercent      = params.DownsamplePercent;
            this.RegistrationGridStep   = params.RegistrationGridStep;
            this.MergeGridStep          = params.MergeGridStep;
            this.Verbose                = params.Verbose;
            
            reset(this);
        end
        
        %------------------------------------------------------------------
        function configureLoopDetector(this, nameValues)
            %configureLoopDetector Configure loop closure detector
            %   configureLoopDetector(mapBuilder) configures the map
            %   builder to detect loop closures.
            %
            %   configureLoopDetector(mapBuilder, Name, Value, ...)
            %   additionally specifies name-value pairs as described below:
            %
            %   'LoopConfirmationRMSE'  RMSE below which a loop closure 
            %                           candidate is confirmed. If the
            %                           RMSE obtained through registration 
            %                           is above this value, the loop 
            %                           closure candidate is rejected.
            %
            %                           Default: 4.5
            %                       
            %   'IgnoreRecentFeatures'  See helperLoopClosureDetector for
            %   'MatchThreshold'        details about these parameters.
            %   'DistanceThreshold'
            %   'UseKdTree'
            %   
            %   See also helperLoopClosureDetector.
            
            arguments
                this(1,1)
                
                nameValues.LoopConfirmationRMSE(1,1) {mustBeNumeric, mustBePositive}= 4.5;
                nameValues.IgnoreRecentFeatures(1,1)                                = 30;
                nameValues.MatchThreshold(1,1)                                      = 0.15;
                nameValues.DistanceThreshold(1,1)                                   = 0.1;
                nameValues.UseKdTree(1,1)                                           = false;
            end
            
            this.DetectLoops = true;
            this.LoopConfirmationRMSE = nameValues.LoopConfirmationRMSE;
            
            loopDetectionParameters = rmfield(nameValues, 'LoopConfirmationRMSE');
            
            params = fieldnames(loopDetectionParameters);
            values = struct2cell(loopDetectionParameters);
            inputs = reshape([params, values]', 1, []);
            this.LoopDetector = helperLoopClosureDetector(inputs{:});
        end
        
        %------------------------------------------------------------------
        function tform = updateMap(this, ptCloudIn, varargin)
            %updateMap Update map with new point cloud
            %   updateMap(mapBuilder, ptCloudIn) updates the map with new
            %   point cloud ptCloudIn. ptCloudIn must be a pointCloud
            %   object.
            %
            %   updateMap(mapBuilder, ptCloudIn, tformInit) updates the map
            %   with new point cloud ptCloudIn using an initial
            %   transformation tformInit as the initial transformation.
            %   tformInit must be a rigid3d object specifying a rigid
            %   transformation. When left unspecified, the previous
            %   transformation is used as the initial transformation. This
            %   effectively assumes a constant velocity.
            %
            %   tform = updateMap(...) additionally returns the
            %   transformation from ptCloudIn to the last provided point
            %   cloud.
            %
            %   See also rigid3d, affine3d, pcregisterndt.
            
            if nargin>2
                tformInit = varargin{1};
            else
                if ~this.IsInitialized
                    tformInit = rigid3d();
                else
                    tformInit = this.PreviousTform;
                end
            end
            
            validateattributes(ptCloudIn, {'pointCloud'}, {'scalar'}, ...
                'updateMap', 'ptCloudIn');
            
            validateattributes(tformInit, {'rigid3d'}, {'scalar'}, ...
                'updateMap', 'ptCloudIn');
            
            % If the pointCloud has no points, ignore it
            hasNoPoints = all( isnan(ptCloudIn.Location), 'all' );
            if hasNoPoints
                tform = rigid3d;
                return;
            end
            
            % Segment ground and remove ego vehicle
            ptCloud = processPointCloud(this, ptCloudIn);
            ptCloud = pcdownsample(ptCloud, 'random', this.DownsamplePercent);
            
            origin = [0 0 0];
            
            if ~this.IsInitialized
                % Initialize Moving and Map pointClouds
                this.Moving = ptCloud;
                this.Map    = ptCloud;
                this.ViewId = 1;
                this.RelativePositions = origin;
                
                % Add first view to the view set
                this.ViewSet = addView(this.ViewSet, this.ViewId, ...
                    this.AccumTform, 'PointCloud', ptCloudIn);
                this.ViewId = this.ViewId + 1;
                
                this.IsInitialized = true;
                tform = rigid3d;
            else
                this.Fixed  = this.Moving;
                this.Moving = ptCloud;
                
                % Register moving point cloud w.r.t fixed
                firstIteration = size(this.RelativePositions,1)==1;
                if firstIteration
                    % Use more iterations for the first frame
                    tform = pcregisterndt(this.Moving, this.Fixed, ...
                        this.RegistrationGridStep, 'InitialTransform', ...
                        tformInit, 'MaxIterations', 50);
                else
                    tform = pcregisterndt(this.Moving, this.Fixed, ...
                        this.RegistrationGridStep, 'InitialTransform', tformInit);
                end
                    
                % Compute accumulated transformation to origin reference
                % frame
                this.AccumTform = rigid3d( tform.T * this.AccumTform.T );
                
                % Add current point cloud as a view to the view set
                this.ViewSet = addView(this.ViewSet, this.ViewId, ...
                    this.AccumTform, 'PointCloud', ptCloudIn);
                
                % Add a connection from the previous view to the current
                % view representing the relative transformation between
                % them.
                this.ViewSet = addConnection(this.ViewSet, this.ViewId-1, ...
                    this.ViewId, tform);
                
                if this.DetectLoops
                    % Detect loop closure candidates
                    [loopFound, loopViewId] = detectLoop(this.LoopDetector, ptCloudIn);
                    
                    if loopFound
                        loopViewId = loopViewId(1);
                        
                        % Retrieve point cloud from view set
                        ptCloudIn = this.ViewSet.Views.PointCloud( find(this.ViewSet.Views.ViewId == loopViewId, 1) );
                        
                        % Segment ground and remove ego vehicle
                        ptCloudOld = processPointCloud(this, ptCloudIn);
                        ptCloudOld = pcdownsample(ptCloudOld, 'random', this.DownsamplePercent);
                        
                        [relTform, ~, rmse] = pcregisterndt(ptCloudOld, ...
                            ptCloud, this.RegistrationGridStep, 'MaxIterations', 50);
                        
                        
                        if this.Verbose
                            fprintf('Loop closure candidate found between view Id %d and %d with RMSE %f...\n', ...
                                this.ViewId, loopViewId, rmse);
                        end
                        
                        acceptLoopClosure = rmse <= this.LoopConfirmationRMSE;
                        if acceptLoopClosure
                            % For simplicity, use a constant, small
                            % information matrix for loop closure edges
                            
                            infoMat = 0.01 * eye(6);
                            %infoMat = eye(6);
                            
                            % Add a connection corresponding to a loop
                            % closure
                            this.ViewSet = addConnection(this.ViewSet, ...
                                loopViewId, this.ViewId, invert(relTform), infoMat);
                            if this.Verbose
                                fprintf('Accepted\n');
                            end
                        else
                            if this.Verbose
                                fprintf('Rejected\n');
                            end
                        end
                        
                    end
                end
                
                % Compute current position relative to origin
                this.RelativePositions(end+1,:) = transformPointsForward(...
                    this.AccumTform, origin);
                
                % Transform moving point cloud to original reference frame
                movingReg = pctransform(this.Moving, this.AccumTform);
                
                % Merge into accumulated map
                this.Map = pcmerge(this.Map, movingReg, this.MergeGridStep);
                
                this.PreviousTform = tform;
                
                this.ViewId = this.ViewId + 1;
            end
        end
        
        %------------------------------------------------------------------
        function isDisplayOpen = updateDisplay(this, closeDisplay)
            %updateDisplay Update display of map builder
            %   isDisplayOpen = updateDisplay(mapBuilder, closeDisplay)
            %   updates the display of mapBuilder. If closeDisplay is true,
            %   the displays are shut down. isDisplayOpen returns a logical
            %   scalar specifying whether the display is open or closed.
            %
            %   See also scatter, pcshow.
            
            if closeDisplay
                closeFigureDisplay(this);
                isDisplayOpen = false;
                return;
            end
            
            if isempty(this.Axes) || ~isvalid(this.Axes)
                setupFigureDisplay(this);
            end
            
            xyzPoints           = this.Map.Location;
            relativePositions   = this.RelativePositions;
            hasIntensity        = ~isempty(this.Map.Intensity);
            
            if isempty(xyzPoints)
                isDisplayOpen = isvalid(this.Axes);
                return;
            end
            
            % Update map display
            if hasIntensity
                % Color points using intensity
                set(this.ScatterPoints, 'XData', xyzPoints(:,1), 'YData', ...
                    xyzPoints(:,2), 'CData', this.Map.Intensity);
            else
                % Color points using Z
                set(this.ScatterPoints, 'XData', xyzPoints(:,1), 'YData', ...
                    xyzPoints(:,2), 'CData', xyzPoints(:,3));
            end
            
            % Update trajectory on map
            set(this.ScatterTrajectory, 'XData', relativePositions(:, 1), ...
                'YData', relativePositions(:,2));
            
            % Limit graphics render rate
            drawnow limitrate
            
            isDisplayOpen = isvalid(this.Axes);
        end
        
        %------------------------------------------------------------------
        function optimizeMapPoses(this, varargin)
            %optimizeMapPoses Optimize map poses using pose graph optimization
            %   optimizeMapPoses(mapBuilder) optimizes map poses using pose
            %   graph optimization and updates the view set.
            %
            %   See also createPoseGraph, optimizePoses.
            
            if this.Verbose
                fprintf('Optimizing pose graph...');
            end
            
            % Create pose graph
            G = createPoseGraph(this.ViewSet);
            
            % Optimize pose graph
            optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
            
            % Update view set views with optimized poses
            this.ViewSet = updateView(this.ViewSet, optimG.Nodes);
            
            if this.Verbose
                fprintf('done\n');
            end
        end
        
        %------------------------------------------------------------------
        function rebuildMap(this, radius, NameValues)
            %rebuildMap Rebuild point cloud map from view set
            %   rebuildMap(mapBuilder) uses point clouds and absolute poses
            %   from the pcviewset object in the ViewSet property of the
            %   mapBuilder to rebuild the point cloud map.
            %
            %   rebuildMap(mapBuilder, radius) additionally specifies a
            %   radius around each point cloud that is used to build the
            %   map.
            %
            %   rebuldMap(..., 'SkipFrames', skipFrames) additionally
            %   specifies the number of frames to skip while rebuilding the
            %   map.
            %
            %   See also pcviewset, pcmerge.
            
            arguments
                this(1,1)
                radius(1,1) double {mustBeNumeric,mustBePositive}                                = 25;
                
                NameValues.SkipFrames(1,1) double {mustBeNumeric, mustBePositive, mustBeInteger} = 2;
            end
            
            skipFrames = NameValues.SkipFrames;
            
            vSet = this.ViewSet;
            
            ptClouds  = vSet.Views.PointCloud(1 : skipFrames : end);
            absTforms = vSet.Views.AbsolutePose(1 : skipFrames : end);
            
            if this.Verbose
                fprintf('Rebuilding map...');
            end
            
            numFrames         = numel(ptClouds);
            downsamplePercent = this.DownsamplePercent;
            mergeGridStep     = this.MergeGridStep;
            for n = 1 : numFrames
                ptCloud = pcdownsample(this.processPointCloud(ptClouds(n)), 'random', downsamplePercent);
                ptClouds(n) = select(ptCloud, findNeighborsInRadius(ptCloud, [0 0 0], radius));
            end
            
            this.Map = pcalign(ptClouds, absTforms, mergeGridStep);
            
            relPositions = arrayfun(@(poseTform) transformPointsForward(poseTform, [0 0 0]), this.ViewSet.Views.AbsolutePose, 'UniformOutput', false);
            this.RelativePositions = vertcat(relPositions{:});
            
            if this.Verbose
                fprintf('done\n');
            end
        end
        
        %------------------------------------------------------------------
        function visualizeFlythrough(this)
            %visualizeFlythrough Visualize flythrough
            %   visualizeFlythrough(mapBuilder) visualize ego-centric
            %   fly-through along path of the built map.
            %
            %   See also campos, camproj, camva.
            
            % Display accumulated point cloud map
            hAx = pcshow(this.Map, 'MarkerSize', 10);
            
            % Turn off axis display
            axis off
            
            % Add the traversed path
            hold(hAx, 'on')
            relativePositions = this.RelativePositions;
            plot3(hAx, relativePositions(:,1), relativePositions(:,2), relativePositions(:,3), ...
                'MarkerSize', 10, 'Color', [0.9 0.9 0.9], 'LineWidth', 3);
            
            % Set camera properties
            camproj(hAx, 'perspective')
            camva(hAx, 90);
            
            numFrames = size(relativePositions, 1);
            lookAhead = 5;
            viewRate  = 20;
            for n = 1 : numFrames-lookAhead
                currentPoint   = relativePositions(n, :);
                lookAheadPoint = relativePositions(n+lookAhead, :);
                
                % Position viewing camera at current pose
                campos(hAx, currentPoint);
                
                % Target viewing camera at look-ahead pose
                camtarget(hAx, lookAheadPoint);
                
                pause(1 / viewRate);
            end
        end
        
        %------------------------------------------------------------------
        function reset(this)
            %reset Reset map builder
            %   reset(mapBuilder) resets the map builder and clears the
            %   built map. The next call to updateMap will begin building a
            %   new map.
            
            this.IsInitialized = false;
            
            this.Fixed  = pointCloud(zeros(0, 0, 3, 'single'));
            this.Moving = pointCloud(zeros(0, 0, 3, 'single'));
            this.Map    = pointCloud(zeros(0, 0, 3, 'single'));
            
            this.ViewSet = pcviewset;
            
            this.LoopDetector       = [];
            this.RelativePositions  = zeros(0, 3);
            this.AccumTform         = rigid3d;
            
            this.Axes = [];
        end
        
        %------------------------------------------------------------------
        function ptCloud = processPointCloud(this, ptCloudIn)
            %processPointCloud Process pointCloud to remove ground and ego vehicle
            %   ptCloud = processPointCloud(mapBuilder, ptCloudIn)
            %   processes the pointCloud ptCloudIn by removing ground plane
            %   and the ego vehicle. Use this method to look at the
            %   processed point cloud used for pointCloud registration.
            %
            %   See also pcfitplane, pointCloud/findNeighborsInRadius.
            
            % Segment ground as the dominant plane with reference normal
            % vector pointing in positive z-direction.
            groundRemovalParams = this.GroundRemovalParameters;
            maxDistance         = groundRemovalParams.MaxDistance;
            maxAngularDistance  = groundRemovalParams.MaxAngularDistance;
            
            referenceVector = [0 0 1];
            
            [~, groundFixedIdx] = pcfitplane(ptCloudIn, maxDistance, ...
                referenceVector, maxAngularDistance);
            
            isOrganized = ~ismatrix(ptCloudIn.Location);
            
            if isOrganized
                groundFixed = false(size(ptCloudIn.Location,1),size(ptCloudIn.Location,2));
            else
                groundFixed = false(ptCloudIn.Count, 1);
            end
            groundFixed(groundFixedIdx) = true;
            
            % Segment ego vehicle as points within a given radius of sensor
            sensorLocation = [0 0 0];
            radius = 3.5;
            egoFixedIdx = findNeighborsInRadius(ptCloudIn, sensorLocation, radius);
            
            if isOrganized
                egoFixed = false(size(ptCloudIn.Location,1),size(ptCloudIn.Location,2));
            else
                egoFixed = false(ptCloudIn.Count, 1);    
            end
            egoFixed(egoFixedIdx) = true;
            
            % Retain subset of point cloud without ground and ego vehicle
            if isOrganized
                indices = ~groundFixed & ~egoFixed;
            else
                indices = find(~groundFixed & ~egoFixed);
            end
            
            ptCloud = select(ptCloudIn, indices);
        end
        
        %------------------------------------------------------------------
        function delete(this)
            
            % Close the parent figure
            closeFigureDisplay(this);
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function setupFigureDisplay(this)
            % Create a figure
            hFig = figure('Name', 'Accumulated Point Cloud Map', ...
                'NumberTitle', 'off', 'HandleVisibility', 'callback');
            
            % Place an axes on the figure
            this.Axes = axes('Parent', hFig);
            
            % Create a scatter object for map points
            mapMarkerSize = 6;
            this.ScatterPoints = scatter(this.Axes, NaN, NaN, ...
                mapMarkerSize, NaN, '.');
            
            hold(this.Axes, 'on');
            
            % Create a scatter object for relative positions
            positionMarkerSize = 25;
            this.ScatterTrajectory = scatter(this.Axes, NaN, NaN, ...
                positionMarkerSize, 'w', 'filled');
            
            hold(this.Axes, 'off');
            
            % Set background color
            this.Axes.Color = 'k';
            this.Axes.Parent.Color = 'k';
            
            % Set labels
            xlabel(this.Axes, 'X (m)')
            ylabel(this.Axes, 'Y (m)')
            
            % Set grid colors
            this.Axes.GridColor = 'w';
            this.Axes.XColor    = 'w';
            this.Axes.YColor    = 'w';
            
            % Set aspect ratio for axes
            axis(this.Axes, 'equal');
            
            title(this.Axes, 'Accumulated Point Cloud Map', 'Color', [1 1 1])
        end
        
        %------------------------------------------------------------------
        function closeFigureDisplay(this)
            if ~isempty(this.Axes) && isvalid(this.Axes)
                close(this.Axes.Parent);
            end
        end
    end
end

%--------------------------------------------------------------------------
function params = parseInputs(varargin)

% Define defaults
defaults.DownsamplePercent              = 0.1;
defaults.RegistrationGridStep           = 3;
defaults.MergeGridStep                  = 0.5;
defaults.Verbose                        = false;

% Parse inputs
parser = inputParser;
parser.FunctionName = mfilename;

parser.addParameter('DownsamplePercent',            defaults.DownsamplePercent);
parser.addParameter('RegistrationGridStep',         defaults.RegistrationGridStep);
parser.addParameter('MergeGridStep',                defaults.MergeGridStep);
parser.addParameter('Verbose',                      defaults.Verbose);

parse(parser, varargin{:});

params = parser.Results;

% Validate inputs
validateattributes(params.DownsamplePercent, {'single','double'}, ...
    {'real','nonsparse','scalar','>=',0,'<=',1}, mfilename, ...
    'DownsamplePercent');

validateattributes(params.RegistrationGridStep, {'single','double'}, ...
    {'real','nonsparse','scalar','finite','positive'}, mfilename, ...
    'RegistrationGridStep');

validateattributes(params.MergeGridStep, {'single','double'}, ...
    {'real','nonsparse','scalar','finite','positive'}, mfilename, ...
    'MergeGridStep');

validateattributes(params.Verbose, {'double', 'logical'}, ...
    {'binary', 'scalar'}, mfilename, 'Verbose');
end