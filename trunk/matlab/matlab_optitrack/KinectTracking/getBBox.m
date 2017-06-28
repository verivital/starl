function BBox = getBBox(center, radius, type)
% returns a bounding box for the robot. Should be big enough robot won't
% move outside it in the next frame.

global BBoxFactor
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4

% if ARDrone, bbox needs to be 4 times radius, since there are 2 circles
% per side
if type == ARDRONE || type == GHOST2
    x = 4;
    y = 2;
else
    x = 2;
    y = 1;
end

BBox = [[center(1,1) - BBoxFactor * radius*y, center(1,2) - BBoxFactor * radius*y], ...
        [x*BBoxFactor*radius, x*BBoxFactor*radius]];
BBox = round(BBox);