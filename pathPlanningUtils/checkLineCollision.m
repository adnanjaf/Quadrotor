function pathValid = checkLineCollision(map, startXYZ, endXYZ)
%% Function to test whether there is a point that collides or goes out of
% bounds in between two waypoints on a path
%   pathValid = 1 if path is valid

pathValid = 0;

xyDist = sqrt(sum((startXYZ(1:2) - endXYZ(1:2)).^2));
zDist = sqrt(sum((startXYZ(3) - endXYZ(3)).^2));

% Generate intermediate points between start and end positions
numPoints = max(xyDist/map.res_xyz(1), zDist/map.res_xyz(3)) + 2;

XYZPoints = [linspace(startXYZ(1), endXYZ(1), numPoints);...
    linspace(startXYZ(2), endXYZ(2), numPoints);...
    linspace(startXYZ(3), endXYZ(3), numPoints)]';

% Identify limits of intermediate trajectory
Xmax = max(XYZPoints(:,1)); Xmin = min(XYZPoints(:,1));
Ymax = max(XYZPoints(:,2)); Ymin = min(XYZPoints(:,2));
Zmax = max(XYZPoints(:,3)); Zmin = min(XYZPoints(:,3));

% Boundary check
if Xmin < map.bound_xyz(1) || Ymin < map.bound_xyz(2) ||...
        Zmin < map.bound_xyz(3) || Xmax > map.bound_xyz(4) ||...
        Ymax > map.bound_xyz(5) || Zmax > map.bound_xyz(6)
    pathValid = 0;
    return;
end

% Convert to subscripts (to refer to voxel positions)
subs = pos2sub(map,XYZPoints);

checkVoxel = map.occgrid(subs(:,1),subs(:,2),subs(:,3));

% Check for collision with obstacle
allCheckedVoxels = sum(sum(sum(checkVoxel)));
% fprintf('Sum of Checked Voxels: %d \n', allCheckedVoxels);
if  allCheckedVoxels == 0
    pathValid = 1;
else
    return;
end

end