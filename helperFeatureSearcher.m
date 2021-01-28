%helperFeatureSearcher Create feature descriptor searcher
%
%   This is an example helper class that is subject to change or removal in
%   future releases.
%
%   searcher = helperFeatureSearcher() returns a feature searcher object.
%   Use the feature searcher to search for closest feature matches.
%
%   searcher = helperFeatureSearcher(Name,Value) specifies additional
%   name-value pair arguments described below:
%
%   'IgnoreRecentFeatures'      A scalar specifying number of recently
%                               added features to ignore during search. Use
%                               this to avoid false positives from recently
%                               added feature vectors.
%
%                               Default: 30
%
%   'MatchThreshold'            A scalar specifying the distance threshold
%                               allowed for a match. A feature is not
%                               matched if the feature distance is greater
%                               than this threshold.
%
%                               Default: 0.1
%
%   'UseKdTree'                 A logical scalar specifying whether a
%                               Kd-tree should be used to speed up feature
%                               search. Use of Kd-tree requires Statistics
%                               and Machine Learning Toolbox(TM).
%
%                               Defautl: false
%
%   helperFeatureSearcher methods:
%   addFeature  - Add a new feature descriptor to the searcher
%   findClosest - Find closest feature descriptors
%
%   helperFeatureSearcher properties:
%   Features                - Matrix of feature descriptors (read-only)
%   NumFeatures             - Number of feature descriptors (read-only)
%   FeatureLength           - Length of each feature descriptor (read-only)
%   IgnoreRecentFeatures    - Number of recent features to ignore (read-only)
%   MatchThreshold          - Feature match threshold (read-only)
%   UseKdTree               - Flag for use of Kd-tree (read-only)
%
%   See also helperExtractScanContextFeatures, KDTreeSearcher, knnsearch.

% Copyright 2019 The MathWorks, Inc.
classdef helperFeatureSearcher < handle
    
    properties (SetAccess = protected)
        %Features 
        %   Matrix of feature descriptors, specified as a
        %   N-by-D matrix containing N number of D-dimensional feature
        %   descriptors.
        Features
        
        %IgnoreRecentFeatures
        %   Number of recently added features to ignore during search.
        %   Increase this value to avoid spurious matches from nearby
        %   feature vectors derived from the same scene.
        %   
        %   Default: 30
        IgnoreRecentFeatures(1,1) double
        
        %MatchThreshold
        %   Maximum distance threshold allowed for a feature match. A
        %   feature is not matched if the feature distance is greater than
        %   this threshold.
        %
        %   Default: 0.1
        MatchThreshold(1,1) double
        
        %UseKdTree
        %   Logical scalar specifying whether Kdtree is used for feature
        %   search. Use of Kd-tree requires Statistics and Machine Learning
        %   Toolbox(TM).
        %
        %   Default: false
        UseKdTree(1,1) logical
     end
    
    properties (Dependent, SetAccess = protected)
        %NumFeatures
        %   Number of feature descriptors.
        NumFeatures
        
        %FeatureLength
        %   Length of each feature descriptor.
        FeatureLength
    end
    
    methods
        %------------------------------------------------------------------
        function this = helperFeatureSearcher(namedargs)
            
            arguments
                namedargs.IgnoreRecentFeatures (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeReal} = 30;
                namedargs.MatchThreshold       (1,1) {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite}  = 0.1;
                namedargs.UseKdTree            (1,1) {mustBeNumericOrLogical}                                   = false;
            end
            
            this.IgnoreRecentFeatures = namedargs.IgnoreRecentFeatures;
            this.MatchThreshold       = namedargs.MatchThreshold;
            this.UseKdTree            = namedargs.UseKdTree;
        end
        
        %------------------------------------------------------------------
        function this = addFeature(this, featureVector)
            %add Add a feature vector for search
            %   searcher = add(searcher, feature) adds a feature to the
            %   searcher for search.
            
            arguments
                this
                featureVector (:,1) {mustBeNumeric, mustBeReal}
            end
            
            if isempty(this.Features)
                validateattributes(featureVector, {'single', 'double'}, {}, ...
                    'addFeature', 'feature');
                
                this.Features = featureVector.';
            else
                validateattributes(featureVector, {'single', 'double'}, ...
                    {'numel', this.FeatureLength}, 'addFeature', 'feature');
                
                this.Features(end+1, :) = featureVector.';
            end
        end
        
        %------------------------------------------------------------------
        function [indices, dists] = findClosest(this, featureVector, K)
            %findClosest Find closest feature descritor
            %   indices = findClosest(searcher, feature, K) returns indices
            %   of the K closest features in the searcher to the feature
            %   vector feature.
            %
            %   [indices, dists] = findClosest(...) additionally returns
            %   the distances.
            %
            %   Notes
            %   -----
            %   When no match is found, indices are empty.
            %
            %   See also knnsearch.
            
            arguments
                this
                featureVector   (:,1) {mustBeNumeric, mustBeReal}
                K               (1,1) double                        = 1;
            end
            
            validateattributes(featureVector, {class(this.Features)}, ...
                {'numel', this.FeatureLength}, 'findClosest', 'feature');
            
            skipNumFeatures = this.IgnoreRecentFeatures;
            if this.NumFeatures <= max(K, skipNumFeatures)
                indices = double([]);
                dists   = cast([], 'like', featureVector);
                return
            end
            
            if this.UseKdTree
                featureTree = KDTreeSearcher( this.Features(1:end-skipNumFeatures,:) );
            
                [indices, dists] = knnsearch(featureTree, featureVector.', 'K', K);
            else
                [dists, indices] = mink(sum( (this.Features(1:end-skipNumFeatures, :) - featureVector.').^2, 2 ), K);
                indices = indices.';
                dists   = sqrt(dists.');
            end
            
            % Remove bad matches
            badMatches = dists >= this.MatchThreshold;
            indices(badMatches) = [];
            dists(badMatches)   = [];
        end
        
        %------------------------------------------------------------------
        function numFeatures = get.NumFeatures(this)
            
            numFeatures = size(this.Features, 1);
        end
        
        %------------------------------------------------------------------
        function featureLength = get.FeatureLength(this)
            
            featureLength = size(this.Features, 2);
        end
    end
end