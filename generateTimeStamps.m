function [timeStamps, totalTime] = generateTimeStamps(path,speed)
% Function to generate timestamps to be at waypoints such that
% every path segment is flown at an average speed

% Total distance traveled by the path
pathDists = diff(path);
norms = sqrt(sum(pathDists.^2,2));
totalPathDist = sum(norms);

% Calculate total time taken assuming average speed
% Note: Speed is empirically tuned
totalTime = totalPathDist/speed;

pathSegmentLength = sqrt(norms);

% Find time-stamps of each successive waypoint
timeStamps = cumsum(pathSegmentLength*totalTime);
timeStamps = timeStamps/timeStamps(end);
timeStamps = [0; timeStamps]';
timeStamps = timeStamps*totalTime;

end