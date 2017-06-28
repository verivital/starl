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
	% Function determined using calibrate_kinect_settings.m
	r = 1.205e-11  *depth^4 + -1.112e-07 *depth^3 ...
	    + 0.0003868 *depth^2 + -0.6106 *depth + 400.6;
	rmin = max(floor(r-2), 1);
	rmax = ceil(r+3);
    if depth == 4900
        rmin = floor(22.2300 - 3);
        rmax = ceil(22.2300 + 3);
    end

elseif (type == CREATE2)
	% These values were determined by previous found in trackBots.m
	rmin = 25;
    rmax = 35;
	
elseif (type == ARDRONE)
	% Function determined using calibrate_kinect_settings.m
%     r = 5.11e-12 *depth^4 + -4.772e-08 *depth^3 ...
% 	    + 0.0001725 *depth^2 + -0.2964 *depth + 237.2;
% 	rmin = max(floor(r-4), 1);
% 	rmax = ceil(r+4);
	% Current values are from this original function which works better
	r = -1.30011064979532e-12  *depth^4 + 5.68628514193467e-09 *depth^3 ...
		+ 6.45027967553587e-06 *depth^2 - 0.06739000549554*depth + 115.106261326994;
	rmin = max(floor(r-5), 1);
	rmax = ceil(r+5);
    if depth == 4900
        rmin = floor(26.7572 - 3);
        rmax = ceil(26.7572 + 3);
    end
	
elseif (type == THREEDR)
	% TODO: Make this more specific to the THREEDR
	rmin = 0;
    rmax = 0;
	
elseif (type == GHOST2)
	% Function determined using calibrate_kinect_settings.m
	r = -1.322e-11  *depth^4 + 9.788e-08 *depth^3 ...
	    + -0.0002566 *depth^2 + 0.2622 *depth + -39.38;
	rmin = max(floor(r-5), 1);
	rmax = ceil(r+5);
    if depth == 4900
        rmin = floor(25.6530 - 3);
        rmax = ceil(25.6530 + 3);
    end
	
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
