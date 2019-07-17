function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

%% Sample Maps and Parameters
% tic
% Test Empty Map
% map = load_map('./sample_maps/emptyMap.txt', 1, 1, 1);
% start = [2,1,1]; goal = [19,4,5];

% Test 2D Map
% map = load_map('./sample_maps/emptyMap.txt', 1, 6, 1);
% start = [2,1,1]; goal = [19,4,5];
% map.occgrid = map.occgrid(:,:,1);

% Test Map 0
% map = load_map('./sample_maps/map0.txt', 1, 1, 0.3);
% start = [2,1,1]; goal = [19,4,5];

% Test Map 1
% map = load_map('./sample_maps/map1.txt', 0.25, 1, 0.5);
% start = [5,-3,1]; goal = [4,18,3];

% Resolution Failure Case
% map = load_map('./sample_maps/map1.txt', 0.1, 1, 0.3);
% start = [5,-5,5]; goal = [4,19,2];
% plot_path(map,[goal])

% DEBUG: Plot path start/goal to check validity
% path = [start;goal];
% plot_path(map,path)

% map = load_map('maps/map1.txt', 0.1, 1.0, 0.2);
% start = [0.0, -4.9, 0.2];
% goal = [8.0, 18.0, 3.0];

%% Main Dijkstra Code
tic
if nargin < 4
    astar = false;
end

num_expanded = 0;

mapSize = size(map.occgrid);

xRes = map.res_xyz(1); yRes = map.res_xyz(2); zRes = map.res_xyz(3);

% Direction vectors to search for neighbors
% dx = xRes*[1,0,0,-1,0,0];
% dy = yRes*[0,1,0,0,-1,0];
% dz = zRes*[0,0,1,0,0,-1];

% With diagonals
dx = xRes*[1,0,0,-1,0,0,1,-1,1,-1,0,0,0,0,1,-1,1,-1,1,-1,1,1,-1,1,-1,-1];
dy = yRes*[0,1,0,0,-1,0,1,1,-1,-1,1,-1,1,-1,0,0,0,0,1,1,-1,1,-1,-1,1,-1];
dz = zRes*[0,0,1,0,0,-1,0,0,0,0,1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1];

% Map bounds
xMin = map.bound_xyz(1); yMin = map.bound_xyz(2); zMin = map.bound_xyz(3);
xMax = map.bound_xyz(4); yMax = map.bound_xyz(5); zMax = map.bound_xyz(6);

% Get matrix subscripts of start and goal positions
startSubs = pos2sub(map,start);
goalSubs = pos2sub(map,goal);

% Check if start and end points are valid
try
    if map.occgrid(startSubs(1), startSubs(2), startSubs(3)) == 1
        disp("Start point intersects obstacle!");
        return;
    end
catch
    disp("Map Matrix is not 3D");
    return;
end

if map.occgrid(goalSubs(1), goalSubs(2), goalSubs(3)) == 1
    disp("End point intersects obstacle!");
    return;
end

startIndex = pos2ind(map,start);
goalIndex = pos2ind(map,goal);

% Map of costs that excludes all obstacles
unvisitedVoxels = inf(mapSize);
unvisitedVoxels(map.occgrid == 1) = NaN;

% Create map object to match each visited voxel with the parent it came from
neighborParents = containers.Map('KeyType','uint32','ValueType','uint32');

% Set cost of start index to zero
unvisitedVoxels(startSubs(1),startSubs(2),startSubs(3)) = 0;
currentSubX = startSubs(1); currentSubY = startSubs(2); currentSubZ = startSubs(3);
currentIndex = startIndex;
neighborXYZ = zeros(1,3);

pathNotFound = 1;

%% Check Voxels
while pathNotFound
    currentXYZ = ind2pos(map,currentIndex);
    % Shift xyz position to be the center from front left bottom corner to
    % center of the voxel
    currentXYZ = [currentXYZ(1)+xRes/2;currentXYZ(2)+yRes/2;currentXYZ(3)+zRes/2];
    % Loop through all the neighbouring voxels
    for i = 1:size(dx,2)
        neighborX = currentXYZ(1) + dx(i);
        neighborY = currentXYZ(2) + dy(i);
        neighborZ = currentXYZ(3) + dz(i);
        neighborXYZ = [neighborX,neighborY,neighborZ];
        
        % Check if neighbor is within map boundaries
        if neighborX >= xMin && neighborY >= yMin && neighborZ >= zMin &&...
                neighborX < xMax && neighborY < yMax && neighborZ < zMax
            % Get linear index of considered neighbor
            neighborIndex = pos2ind(map,[neighborX,neighborY,neighborZ]);
            neighborSubs = pos2sub(map,[neighborX,neighborY,neighborZ]);
            
            % Check if neighbor is unvisited
            if unvisitedVoxels(neighborSubs(1),neighborSubs(2),neighborSubs(3))~= NaN
                if astar
                    temp_cost_to_neighbor = ...
                        unvisitedVoxels(currentSubX,currentSubY,currentSubZ)...
                        + norm(neighborXYZ-goal,1);
                else
                    temp_cost_to_neighbor = unvisitedVoxels(currentSubX,currentSubY,currentSubZ)+ norm(neighborXYZ-currentXYZ);
                end
                
                % Keep the minimum cost and parent to get to the neighbor
                if temp_cost_to_neighbor < unvisitedVoxels(neighborSubs(1),neighborSubs(2),neighborSubs(3))
                    unvisitedVoxels(neighborSubs(1),neighborSubs(2),neighborSubs(3))...
                        = temp_cost_to_neighbor;
                    neighborParents(neighborIndex) = currentIndex;
                end
            end
        end
    end
    
    % Remove the current node from the unvisited list
    unvisitedVoxels(currentSubX,currentSubY,currentSubZ) = NaN;
    
    % Update number of voxels visited
    minCost = min(min(min(unvisitedVoxels)));
    minSubs=find(unvisitedVoxels==minCost);
    currentIndex = minSubs(1);
    [currentSubX,currentSubY,currentSubZ] = ind2sub(mapSize,currentIndex);
    num_expanded = num_expanded + 1;
    
    % Check to see if there are any viable voxels left to visit
    if minCost == Inf
        disp('No Path Exists')
        disp(['Num Expanded: ',num2str(num_expanded)])
        path = zeros(0,3);
        break;
    end
    
    % Break the loop if you reach the end
    if currentIndex == goalIndex
        disp('The shortest path was found!')
        disp(['It took ',num2str(num_expanded),' steps to find it']);
        % Record total path taken
        pathIndex = goalIndex;
        pathIndices = [];
        pathIndices(1) = goalIndex;
        
        % Recursively find the min. cost path from goal to start
        while pathIndex ~= startIndex
            pathIndices(end+1) = neighborParents(pathIndex);
            pathIndex = neighborParents(pathIndex);
        end
        pathIndices = flip(pathIndices);
        prePath = ind2pos(map,pathIndices);
        % Adjust start and end points to be center of voxel
        %path = [prePath(:,1) + (0.5*xRes),prePath(:,2)+ (0.5*yRes), prePath(:,3)+ (0.5*zRes)];
%         centering = ones(size(prePath,1),3).* map.res_xyz;
%         path = prePath + centering;
        path = [start;prePath;goal];
        toc
        %         plot_path(map,path)
        break;
    end
end

end



