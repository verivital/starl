function [rmin, rmax] = findRadiusRange(depth, type)
% The purpose of this function is to calculate and return the radius range of the desired type of robot in terms of pixels

global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4


if (type == MINIDRONE)
	% TODO: Make this more specific to the MINIDRONE
	% Current values are from this original function
	r = -1.30011064979532e-12  *depth^4 + 5.68628514193467e-09 *depth^3 ...
		+ 6.45027967553587e-06 *depth^2 - 0.06739000549554*depth + 115.106261326994;
	rmin = max(floor(r-5), 1);
	rmax = ceil(r+5);

elseif (type == CREATE2)
	% These values were determined by previous found in trackBots.m
	rmin = 25;
    rmax = 35;
	
elseif (type == ARDRONE)
	% TODO: Make this more specific to the ARDRONE
	% Current values are from this original function
	r = -1.30011064979532e-12  *depth^4 + 5.68628514193467e-09 *depth^3 ...
		+ 6.45027967553587e-06 *depth^2 - 0.06739000549554*depth + 115.106261326994;
	rmin = max(floor(r-5), 1);
	rmax = ceil(r+5);
	
elseif (type == THREEDR)
	% TODO: Make this more specific to the THREEDR
	rmin = 0;
    rmax = 0;
	
elseif (type == GHOST2)
	% TODO: Make this more specific to the GHOST2
	rmin = 0;
    rmax = 0;
	
elseif (type == MAVICPRO)
	% TODO: Make this more specific to the MAVICPRO
	rmin = 0;
    rmax = 0;
	
elseif (type == PHANTOM3)
	% TODO: Make this more specific to the PHANTOM3
	rmin = 0;
    rmax = 0;
	
elseif (type == PHANTOM4)
	% TODO: Make this more specific to the PHANTOM4
	rmin = 0;
    rmax = 0;
	
else
	disp('No valid type of drone entered in findRadiusRange');
	rmin = 0;
    rmax = 0;
	
end
