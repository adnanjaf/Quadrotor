function [desired_state] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent totalTime timeStamps map0 simplifiedPath trajCoeffs

% Tuned average speed of trajectory
speed = 2;

% When called the first time, generate time-stamps and trajectory coeffs
if numel(t) == 0 
    map0 = map;
    simplifiedPath = simplifyPath(map0,path);
    [timeStamps, totalTime] = generateTimeStamps(simplifiedPath,speed);
    trajCoeffs = minSnap(simplifiedPath,timeStamps);
    return
end

% If you are reaching the end, go straight to goal
if t >= totalTime
    pos = simplifiedPath(end,:)';
    vel = [0;0;0];
    acc = [0;0;0];
else
    % Find coeffs for current time-stamp
    ind = find(timeStamps <=t);
    ind = ind(end);
    
    % Calculate min. snap trajectory
    pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*trajCoeffs(8*(ind-1)+1:8*ind,:);
    vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*trajCoeffs(8*(ind-1)+1:8*ind,:);
    acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*trajCoeffs(8*(ind-1)+1:8*ind,:);
end

yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end