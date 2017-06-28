function ans = isAerialDrone(type)
% The purpose of this function is to return whether or not the input drone type is aerial or not

% Only the aerial type drones need to be listed here
global MINIDRONE
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4

% if the type is a match, then the function will return true
if ((type == MINIDRONE) || (type == ARDRONE) || (type == THREEDR) || (type == GHOST2) ...
		|| (type == MAVICPRO) || (type == PHANTOM3) || (type == PHANTOM4))
    ans = 1;
else
    ans = 0;
end
return;