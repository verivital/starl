function  angle  = findYaw( imgColor, BBox, yaw, center, radius, type)
% finds the yaw of the minidrone using yellow and cyan markers
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4

% if type is ardrone, use a tight bounding box since blacking out pixels
% doesn't work with drone made up of four circles
if type == ARDRONE || type == GHOST2
    BBox = getBBoxTight(center, radius, type);
end
% take only pixels in bounding box
frame = getPixelsInBB(imgColor, BBox);

if type == MINIDRONE || type == CREATE2 
    % black out pixels that aren't contained in bot's circle
    % this is so yaw estimation won't pick up other bot's circles when too
    % close
 
    % make matrices with with x and y coordinates as values
    x = (1:640);
    X = repmat(x,480,1);
    X = getPixelsInBB(X, BBox);
    y = (1:480)';
    Y = repmat(y,1,640);
    Y = getPixelsInBB(Y, BBox);
    % make a matrix with with 1's inside circle, 0's outside
    imgfilt = (X - center(1,1)).^2 + (Y - center(1,2)).^2 <= radius^2;
    % make the matrix NxNx3
    imgfilt = repmat(imgfilt,1,1,3);
    % multiple frame by the matrix to black out pixels
    frame = frame .* uint8(imgfilt);
end

red = frame(:,:,1);
green = frame(:,:,2);
blue = frame(:,:,3);

% threshold for magenta
mag_red = red > 120 & red < 200;
mag_green = green < 70;
mag_blue = blue > 60 & blue < 130;
mag_img = mag_red .* mag_green .* mag_blue;

% threshold for yellow
yel_red = red > 140 & red < 240;
yel_green = green > 140 & green < 230;
yel_blue = blue < 110;
yel_img = yel_red .* yel_green .* yel_blue;

% label areas
mag_img_labeled = bwlabel(mag_img);
yel_img_labeled = bwlabel(yel_img);
areas = [];

% find centroids
mag_props = regionprops(mag_img_labeled, 'Area', 'Centroid');
yel_props = regionprops(yel_img_labeled, 'Area', 'Centroid');

% if regionprops were found, find yaw
if ~isempty(mag_props) && ~isempty(yel_props)
    % find centroid from region with largest area
    for i = 1:length(mag_props)
        areas(i) = mag_props(i).Area;
    end
    [~,index] = find(areas == max(areas));
    mag_center = mag_props(index(1)).Centroid(:)';
    
    areas = [];
    for i = 1:length(yel_props)
        areas(i) = yel_props(i).Area;
    end
    [~,index] = find(areas == max(areas));
    yel_center = yel_props(index(1)).Centroid(:)';
    
    A = yel_center - mag_center;
    
    if isAerialDrone(type) == 1
        B = [1,0];
    elseif isGroundRobot(type) == 1
        B = [0,1];
    end
    
    angle = rad2deg(angleBtwVectors(A,B));    
%     figure(2);
%     imshow(frame)
%     hold on
%     plot([mag_center(1),yel_center(1)], [mag_center(2),yel_center(2)], '-xg');
%     text(center(1,1), center(1,2), num2str(angle));
% if regionprops not found, return prev yaw
else
    angle = yaw;
end

end

