function trackBots(imgColor, imgDepth, index)
global botArray
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVIKPRO
global PHANTOM3
global PHANTOM4
global camDistToFloor
global hysteresis

% get pixels in bouding box of bot
depthFrame = getPixelsInBB(imgDepth, botArray(index).BBox);
frame = getPixelsInBB(imgColor, botArray(index).BBox);
depth= 0;
centers = [];
radii = [];
metrics = [];

if botArray(index).type == MINIDRONE
    % get depth and rmin, rmax
    depth = findDepth(depthFrame);
    [rmin, rmax] = findRadiusRange(depth, MINIDRONE);
    % find circles
    [centers, radii, metrics] = imfindcircles(frame, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.92);
elseif botArray(index).type == CREATE2
    rmin = 25;
    rmax = 35;
    % find circles
    [centers, radii, metrics] = imfindcircles(frame, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.96);
elseif botArray(index).type == ARDRONE
    depth = findDepth(depthFrame);
    [rmin, rmax] = findRadiusRange(depth, ARDRONE);
    % find circles
    [centers, radii, metrics] = imfindcircles(frame, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.92);
    % not enough circles found, clear centers so function will return below
    if length(centers) < 4
%         figure();
%         image(frame);
%         hold on
%         viscircles(centers, radii);
%         hold off
        centers = [];
    else
        centers = centers(1:4,:);
        % find mean of 4 circles to get center 
        ARCenters = [centers(1,:);centers(2,:); ...
            centers(3,:); centers(4,:)];
        % find an average radius value
        ARRadii = [radii(1), radii(2), radii(3), ...
            radii(4)];
        centers = mean(ARCenters);
        radii = mean(ARRadii);  
        metrics = 1;
    end
elseif botArray(index).type == GHOST2
    depth = findDepth(depthFrame);
    [rmin, rmax] = findRadiusRange(depth, GHOST2);
    % find circles
    [centers, radii, metrics] = imfindcircles(frame, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.92);
    % not enough circles found, clear centers so function will return below
    if length(centers) < 4
%         figure();
%         image(frame);
%         hold on
%         viscircles(centers, radii);
%         hold off
        centers = [];
    else
        centers = centers(1:4,:);
        % find mean of 4 circles to get center 
        GhostCenters = [centers(1,:);centers(2,:); ...
            centers(3,:); centers(4,:)];
        % find an average radius value
        GhostRadii = [radii(1), radii(2), radii(3), ...
            radii(4)];
        centers = mean(GhostCenters);
        radii = mean(GhostRadii);  
        metrics = 1;
    end
end

% if not found, add current value to accum values and return
if isempty(centers)
    [botArray(index).color, ' bot not found']
    botArray(index).centers = [botArray(index).centers; botArray(index).center];
    botArray(index).depths = [botArray(index).depths, botArray(index).depth];
    botArray(index).radii = [botArray(index).radii, botArray(index).radius];
    botArray(index).yaws = [botArray(index).yaws, botArray(index).yaw];
    botArray(index).hyst = botArray(index).hyst + 1;
    return
end
botArray(index).hyst = 0;
% keep strongest circle, put back in original coordinates
[~, indexCircle] = max(metrics);
% add center to botArray, add BBox to get back in whole image px coord
botArray(index).center(1,1) = centers(indexCircle,1) + max([botArray(index).BBox(1),1]);
botArray(index).center(1,2) = centers(indexCircle,2) + max([botArray(index).BBox(2),1]);
botArray(index).centers = [botArray(index).centers; botArray(index).center];

% add radius to bot array
botArray(index).radius = radii(indexCircle,:);
botArray(index).radii = [botArray(index).radii, radii(indexCircle,:)];

% find bbox
botArray(index).BBox = getBBox(botArray(index).center, botArray(index).radius, botArray(index).type);
botArray(index).BBoxes = [botArray(index).BBoxes; botArray(index).BBox];

% add depth found if minidrone, add dist to floor if create, find yaws
if isAerialDrone(botArray(index).type)
    botArray(index).depth = depth;
    botArray(index).yaw = findYaw(imgColor,  botArray(index).BBox,...
        botArray(index).yaw, botArray(index).center, botArray(index).radius, botArray(index).type);
    %botArray(index).yaw = 0;
elseif isGroundRobot(botArray(index).type)
    botArray(index).depth = camDistToFloor;
    botArray(index).yaw = findYaw(imgColor, botArray(index).BBox, ...
        botArray(index).yaw, botArray(index).center, botArray(index).radius, CREATE2);
end

% add accumulated values
botArray(index).yaws = [botArray(index).yaws, botArray(index).yaw];
botArray(index).depths = [botArray(index).depths, depth];
end




    
    

