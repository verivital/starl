function out = isGenQuadCopter(index, centers, rmin, rmax)
% The purpose of this function is to identify the four corners of a generic, square shaped, quadcopter.
% If the circles cannot be used to construct a quadcopter, a single index is returned.
% If the function is successful, the indeces of all four corners is returned.
% Drones that fit this descrition are:
% GhostDrone2.0
% ARDrone
% Phantom 3 & 4

% Calculate the range of corner distances
cornersRange = 2*[rmin, rmax];

% Calculate the range of diagnol distances
diagRange = 2*sqrt(2)*[rmin, rmax];

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