function out = isGhostDrone(index, centers, rmin, rmax)
% this function returns the index if centers(index) does not belong to a
% GhostDrone 2.0
% it returns the index and three other circles making the drone otherwise

% these are the pixel distance ranges for the circles of an ARDrone on the
% ground
cornersRange = [rmin*1.9, rmax*2.1];
diagRange = sqrt(2)*[rmin*1.9, rmax*2.1];

% if values in this range are found, they are added to corners or diag
corners = [];
diag = [];
for i = 1:length(centers)
    d(i) = norm(centers(index,:) - centers(i,:));
    if d(i) > cornersRange(1) && d(i) < cornersRange(2)
        corners = [corners; i];
    end
    if d(i) > diagRange(1) && d(i) < diagRange(2)
        diag = [diag; i];
    end
end

if length(corners) == 2 && length(diag) == 1
    out = [index; corners(1); corners(2); diag];
else
    out = index;
end