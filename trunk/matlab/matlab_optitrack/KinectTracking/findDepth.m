function depth = findDepth(depthFrame)

depth_pixels = [];
depth_pixels_i = find(depthFrame > 50 & depthFrame < 2900);
for j = 1 : length(depth_pixels_i)
    depth_pixels(j) = depthFrame(depth_pixels_i(j));
end
if isempty(depth_pixels)
    depth = 2900;
else
    % depth = median(depth_pixels);
    % Nate trying something
    depth = min(depth_pixels);
end

% idea: threshold image, then call [r,c,v] = find(X), which should return
% nonzero values