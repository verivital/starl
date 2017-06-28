function ans = isGroundRobot(type)
% the purpose of this function is to return whether or not the input robot type is a ground robot

% Only the ground type robots need to be listed
global CREATE2

% if the type is a match, then the function will return true
if (type == CREATE2)
    ans = 1;

else
    ans = 0;
end

return;