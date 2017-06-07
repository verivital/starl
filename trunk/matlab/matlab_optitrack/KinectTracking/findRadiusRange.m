function [rmin, rmax] = findRadiusRange(depth)
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVIKPRO
global PHANTOM3
global PHANTOM4

r = -1.30011064979532e-12  *depth^4 + 5.68628514193467e-09 *depth^3 ...
    + 6.45027967553587e-06 *depth^2 - 0.06739000549554*depth + 115.106261326994;
rmin = max(floor(r-5), 1);
rmax = ceil(r+5);