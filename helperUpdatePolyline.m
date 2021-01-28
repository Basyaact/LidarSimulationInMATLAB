function helperUpdatePolyline(hPoly, x, y)
%helperUpdatePolyline Update polyline position
%   helperUpdatePolyline updates the polyline ROI object hPoly to add a
%   point at [x,y].
%
%   See also images.roi.Polyline

% Copyright 2019 The MathWorks, Inc.

hPoly.Position(end+1,:) = [x y];
end