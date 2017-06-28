function found = findBots(imgColor, imgDepth)
global numDrones
global numARDrones
global numCreates
global num3DRDrones
global numGhostDrones
global numMavicDrones
global numPhant3Drones
global numPhant4Drones
global botArray
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4
% this function calls imfindcircles on whole image to find all robots
% if not enough circles found, it returns false

% The sorting and identification of robots occurs in order of increasing radius
% Robots with smallest search circles are found first

% find any drones
if numDrones + numARDrones > 0
    % find depth
    depth = 2900;
    % find circles
    [rmin, rmax] = findRadiusRange(depth, ARDRONE);
    [centers, radii, metrics] = imfindcircles(imgColor, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.92);
    
     % if not enough circles found, return
    if length(radii) < numDrones + numARDrones*4
        found = false;
        return
    end
    
    % if there are no ARDrones, just look for minidrones
    if numARDrones == 0
        for i = 1:numDrones
            [~, index] = max(metrics);
            botArray(i).center = centers(index,:);
            botArray(i).radius = radii(index);
            botArray(i).BBox = getBBox(centers(index,:), radii(index), MINIDRONE);
            botArray(i).color = getColor(imgColor, centers(index,:));
            botArray(i).type = MINIDRONE;
            botArray(i).yaw = 0;
            botArray(i).hyst = 0;
            % remove selected circle
            centers(index,:) = [];
            radii(index) = [];
            metrics(index) = [];
        end
        
    % otherwise look for both types
    else
        % get the strongest circles
        centers = centers(1:numDrones + numARDrones*4,:);
        radii = radii(1:numDrones + numARDrones*4);
        % sort by size, the minidrone circles will be smaller
        [radii_sorted, I] = sort(radii);
        centers_sorted = [];
        for k = 1:numDrones + numARDrones*4
            centers_sorted(k,:) = centers(I(k),:);
        end
        %divide circles between mindrones and ARdrones
        centers_mini = centers_sorted(1:numDrones,:);
        radii_mini = radii_sorted(1:numDrones);
        centers_ar = centers_sorted(numDrones + 1:numDrones + numARDrones*4,:);
        radii_ar = radii_sorted(numDrones + 1:numDrones + numARDrones*4);
        
        %add minidrones to botArray, which are the smaller cirlces
        for i = 1:size(centers_mini,1)
            botArray(i).center = centers_mini(i,:);
            botArray(i).radius = radii_mini(i);
            botArray(i).BBox = getBBox(botArray(i).center, botArray(i).radius, MINIDRONE);
            botArray(i).color = getColor(imgColor, botArray(i).center);
            botArray(i).type = MINIDRONE;
            botArray(i).yaw = 0;
            botArray(i).hyst = 0;
        end
        
        % add ARDrones, which are the bigger circles
        % make array to track which circles have been used
        circlesUsed = 1:numARDrones*4;
        % check circles until all circles have been used
        i = numDrones + 1;
        while any(circlesUsed)
            index = 1;
            % get the first unused circle
            while ~circlesUsed(index)
                index = index + 1;
            end
            % determine if the circle belongs to an ARDrone
            isAR = isARDrone(index, centers_ar);
            if length(isAR) == 4
                % this is an ARDrone
                % find center by finding mean of 4 circles
                ARCenters = [centers_ar(isAR(1),:);centers_ar(isAR(2),:); ...
                    centers_ar(isAR(3),:); centers_ar(isAR(4),:)];
                % find an average radius value
                ARRadii = [radii_ar(isAR(1)), radii_ar(isAR(2)), radii_ar(isAR(3)), ...
                    radii_ar(isAR(4))];
                botArray(i).center = mean(ARCenters);
                botArray(i).radius = mean(ARRadii);
                botArray(i).type = ARDRONE;
                botArray(i).BBox = getBBox(botArray(i).center, botArray(i).radius, ARDRONE);
                botArray(i).color = getColor(imgColor, botArray(i).center);
                botArray(i).yaw = 0;
                botArray(i).hyst = 0;
                i = i + 1;
            else 
                'error: circle does not belong to ARDrone'
                found = false;
                return 
            end
            % make circles that were used be zero
            for j = 1:length(isAR)
                circlesUsed(isAR(j)) = 0;
            end
        end
    end
end

% find any creates
if numCreates > 0
    [rmin, rmax] = findRadiusRange(depth, CREATE2);
    [centers, radii, metrics] = imfindcircles(imgColor, [rmin,rmax], ...
        'ObjectPolarity', 'dark', 'Sensitivity', 0.92);
    
    % if not enough circles found, return
    if length(radii) < numCreates
        found = false;
        return
    end
    
    % add position information from strongest circles to botArray
    for i = numDrones + numARDrones + 1 : numCreates + numDrones + numARDrones
        [~, index] = max(metrics);
        botArray(i).center = centers(index,:);
        botArray(i).radius = radii(index);
        botArray(i).BBox = getBBox(centers(index,:), radii(index), CREATE2);
        botArray(i).color = getColor(imgColor, centers(index,:));
        botArray(i).type = CREATE2;
        botArray(i).yaw = 0;
        botArray(i).hyst = 0;
        % remove selected circle
        centers(index,:) = [];
        radii(index) = [];
        metrics(index) = [];
    end     
end

found = true;

if numCreates + numDrones + numARDrones == 2
    if (botArray(1).color == 'w' || botArray(2).color == 'w')
        found = false;
    end
end

% this checks to make sure only bot is white
% the first frame is sometimes very bright, so findBots needs to be called
% again on a better frame
num_whites = 0;
for i = 1:numCreates + numDrones + numARDrones
    if botArray(i).color == 'w'
        num_whites = num_whites + 1;
    end
end

if num_whites > 1
    found = false;
end

    