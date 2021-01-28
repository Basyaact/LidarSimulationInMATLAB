%helperLoopClosureDetector Helper class to detect loop closures
%
%   This is an example helper class that is subject to change or removal in
%   future releases.
%
%   helperLoopClosureDetector detects loop closures from point cloud data
%   using Scan Context feature descriptors and a two-level Kd-tree based
%   nearest neighbor feature search.
%
%   loopDetector = helperLoopClosureDetector() returns a loop closure
%   detector object.
%
%   loopDetector = helperLoopClosureDetector(...,Name,Value) specifies
%   additional name value pair arguments as described below:
%
%   'IgnoreRecentFeatures'  A scalar specifying the number of recently
%                           added features to ignore while searching for a
%                           loop closure. Use this to avoid false positives
%                           from recently added feature vectors.
%
%                           Default: 30
%
%   'MatchThreshold'        A scalar specifying the threshold allowed for a
%                           match. A feature is not matched if the feature
%                           distance is greater than this value.
%
%                           Default: 0.1
%
%   'UseKdTree'            A logical scalar specifying whether a Kd-tree
%                          should be used to speed up feature search. Use
%                          of Kd-tree requires Statistics and Machine
%                          Learning Toolbox(TM).
%
%                          Default: false
%
%   helperLoopClosureDetector methods:
%   detectLoop      - Detect loop closure.
%
%   helperLoopClosureDetector properties:
%   FeatureSearcher - A feature searcher object (read-only)
%   ScanContexts    - Scan context features (read-only)
%   UseKdTree       - Flag for use of Kd-tree (read-only)
%   
%   See also scanContextDescriptor, helperFeatureSearcher.

% Copyright 2019-2020 The MathWorks, Inc.
classdef helperLoopClosureDetector < handle
    
    properties (SetAccess = protected)
        %FeatureSearcher
        %   A helperFeatureSearcher object used to find closest feature
        %   matches.
        FeatureSearcher(1,1) helperFeatureSearcher
        
        %ScanContexts
        %   A M-by-N-P matrix of scan context feature images corresponding
        %   to the P point clouds added as part of loop detection.
        ScanContexts
        
        %DistanceThreshold
        %   A scalar between 0 and 1 specifying the distance threshold used
        %   for descriptor matching. See scanContextDistance.
        DistanceThreshold = 0.1;
        
        %UseKdTree
        %   A logical scalar specifying whether a Kd-tree should be used to
        %   speed up feature search. Use of Kd-tree requires Statistics and
        %   Machine Learning Toolbox(TM).
        UseKdTree(1,1) logical
    end
    
    methods
        %------------------------------------------------------------------
        function this = helperLoopClosureDetector(namedargs)
            
            arguments
                namedargs.IgnoreRecentFeatures (1,1) double     {mustBeInteger, mustBePositive, mustBeReal} = 30;
                namedargs.MatchThreshold       (1,1) double     {mustBePositive, mustBeReal}                = 0.1;
                namedargs.DistanceThreshold    (1,1) double     {mustBePositive, mustBeReal}                = 1;
                namedargs.UseKdTree            (1,1) logical    {mustBeNumericOrLogical}                    = false;
            end
            
            % Create a feature searcher
            this.FeatureSearcher = helperFeatureSearcher(...
                'IgnoreRecentFeatures', namedargs.IgnoreRecentFeatures, ...
                'MatchThreshold',       namedargs.MatchThreshold, ...
                'UseKdTree',            namedargs.UseKdTree);
            
            this.DistanceThreshold  = namedargs.DistanceThreshold;
            this.UseKdTree          = namedargs.UseKdTree;
        end
        
        %------------------------------------------------------------------
        function [loopFound, loopIndices] = detectLoop(this, ptCloud)
            
            arguments
                this    (1,1)
                ptCloud (1,1) pointCloud
            end
            
            % Extract scan context descriptor for loop closure detection
            scanContext = scanContextDescriptor(ptCloud);
            
            % Compute ring key feature vector for 1st level of fast search
            numSectors = size(scanContext, 2);
            featureVector = (numSectors - sum(isnan(scanContext),2)) / numSectors;
            
            if isempty(this.ScanContexts)
                this.ScanContexts = scanContext;
            else
                this.ScanContexts(:, :, end+1) = scanContext;
            end
            
            % Add feature vector to feature search
            addFeature(this.FeatureSearcher, featureVector);
            
            % Find indices of closest features
            numNeighbors = 30;
            loopIndices = findClosest(this.FeatureSearcher, featureVector, numNeighbors);
            
            if ~isempty(loopIndices)
                K = 1;
                
                dists = nan(1, numel(loopIndices));
                for n = 1 : numel(loopIndices)
                    dists(n) = scanContextDistance(this.ScanContexts(:, :, loopIndices(n)), scanContext);
                end
                [distVals, idx] = mink(dists, K);
                idx = idx(distVals <= this.DistanceThreshold);
                loopIndices = loopIndices(idx);
            end
            
            loopFound = ~isempty(loopIndices);
        end
    end
end