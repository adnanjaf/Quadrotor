function simplifiedPath = simplifyPath(map, path)
%% Function to remove inefficient waypoints from path
% (e.g. points that are collinear, points that can be skipped via a
% diagonal, etc.)

% Verify path data type
if iscell(path)
    path = cell2mat(path{1:end});
else
    path = path(1:end,:);
end

% Remove duplicate start positions
if path(1,:) == path(2,:)
    path = path(2:end,:);
end

% Remove duplicate end positions
if path(end,:) == path(end-1,:)
    path = path(1:end-1,:);
end

% Offset path positions to be center of voxel (as opposed to bottom left
% back corner)
path(2:end-1,:) = path(2:end-1,:)+(map.res_xyz/2);


% Initialize logical array to sort waypoints
pointsToKeep = false(size(path,1),1);
% Keep start and end points
pointsToKeep(1) = 1;
pointsToKeep(end) = 1;
lastKeptPoint = path(1,:);

% Iterate through all intermediate points in the path
for i = 2:size(path,1)-1
    % Check if there is a clear path to the subsequent waypoint
    subsequentPathValid = checkLineCollision(map, lastKeptPoint, path(i+1,:));
    
%     % Debugging
%     scatter3(lastKeptPoint(1),lastKeptPoint(2),lastKeptPoint(3),100,'filled')
    
    % If the next point is valid and you cannot jump straight to the
    % subsequent point, keep the next point
    if ~subsequentPathValid
        pointsToKeep(i) = 1;
        lastKeptPoint = path(i,:);
    end
end 

simplifiedPath = path(pointsToKeep,:);
plot_path(map,simplifiedPath);

numWaypoints = size(simplifiedPath,1);

% If there aren't enough points, the trajectories can stray out of
% bounds/into obstacles!

% Linearly interpolate between waypoints to properly constrain trajectory
if numWaypoints < 4
    N = 3;
    x = 1:numWaypoints;
    xp = linspace(1,numWaypoints, N*(numWaypoints-1)+N);
    simplifiedPath = interp1(x,simplifiedPath,xp,'linear');
end

scatter3(simplifiedPath(:,1),simplifiedPath(:,2),simplifiedPath(:,3),100,'filled');

end









