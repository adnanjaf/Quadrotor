function X = minSnap(path,timeStamps)
% Function to optimize minimum snap trajectory by solving 8m linear
% equations

% Referenced and adapted from: https://github.com/yrlu/quadrotor/blob/master/traj_planning/traj_opt7.m

% Inputs
% path: (m+1) x 3 matrix of path waypoint coordinates
% timeStamps: mx1 vector containing time-stamps of each waypoint

% Output
% X: 8m x 3 solution vector

[waypts,xyz] = size(path);
numSegments = waypts - 1;
% There are m+1 points in the path

ts = timeStamps;
% ts(k) = t_{k+1}, e.g. ts(1) = t_0,

% Constraints
A = zeros(8*numSegments, 8*numSegments);
% Solution vector for equations
b = zeros(8*numSegments,xyz);
% Trajectory coefficients (to be solved for)
X = zeros(8*numSegments,xyz);

A(:,:) = eye*(8*numSegments)*eps;

idx = 1; % Initialize constraint row counter

%% Constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k
% Continuity requires start and end positions of consecutive trajectories
% to be the same

% E.g. x_1(t_1) = x_2(t_1) = p_1;
% This gives 2*(m-1) constraints in total
for k = 1:numSegments-1
    % Constraints for starting position
    % Coeffs are 7th-order polynomial of time-stamps
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),0);
    b(idx,:) = path(k+1,:);
    idx = idx + 1;
    % Constraints for next position
    A(idx, 8*(k)+1:8*(k+1)) = polyVect(ts(k+1),0);
    b(idx,:) = path(k+1,:);
    idx = idx + 1;
end


%% Constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
% Continuity requires start and end velocities of consecutive trajectories
% to be the same

% E.g. \dot{x}_1(t_1) = \dot{x}_2(t_1)
% This gives m-1 constraints in total
for k = 1:numSegments-1
    % Constraints for starting and ending velocities
    % Coeffs are 1st derivative of 7th-order polynomial of time-stamps
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),1);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),1);
    idx = idx + 1;
end

%% Constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
% Continuity requires start and end accelerations of consecutive trajectories
% to be the same

% e.g. \ddot{x}_1(t_1) = \ddot{x}_2(t_1)
% This gives m-1 constraints in total

for k = 1:(numSegments-1)
    % Constraints for starting and ending accelerations
    % Coeffs are 2nd derivative of 7th-order polynomial of time-stamps
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),2);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),2);
    idx = idx + 1;
end

%% Constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
% Continuity requires start and end jerks of consecutive trajectories
% to be the same

% E.g. x^(3)_1(t_1) = x^(3)_2(t_1)
% This gives m-1 constraints in total
for k = 1:(numSegments-1)
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),3);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),3);
    idx = idx + 1;
end

%% Constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
% Continuity requires start and end snaps of consecutive trajectories
% to be the same

% E.g. x^(4)_1(t_1) = x^(4)_2(t_1)
% This gives m-1 constraints in total
for k = 1:(numSegments-1)
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),4);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),4);
    idx = idx + 1;
end

%% Constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
% Continuity requires start and end crackles of consecutive trajectories
% to be the same

% E.g. x^(5)_1(t_1) = x^(5)_2(t_1)
% This gives m-1 constraints in total

for k = 1:(numSegments-1)
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),5);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),5);
    idx = idx + 1;
end

%% Constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
% Continuity requires start and end pops of consecutive trajectories
% to be the same

% E.g. x^(6)_1(t_1) = x^(6)_2(t_1)
% This gives m-1 constraints in total
for k = 1:(numSegments-1)
    A(idx, 8*(k-1)+1:8*k) = polyVect(ts(k+1),6);
    A(idx, 8*(k)+1:8*(k+1)) = -polyVect(ts(k+1),6);
    idx = idx + 1;
end

%% Initial Position Constraints
% Trajectory starts at the first waypoint at rest
%    x_1(t_0) = p_0
%    x^(1)_0(t_0) = 0
%    x^(2)_0(t_0) = 0
%    x^(3)_0(t_0) = 0
k = 1;
A(idx:idx+3, 8*(k-1)+1:8*k) = [polyVect(ts(k),0);...
                                polyVect(ts(k),1); 
                                polyVect(ts(k),2);
                                polyVect(ts(k),3)];
b(idx,:) = path(k,:); 
idx = idx + 4;

%% Final Waypoint Constraints
% Trajectory ends at the last waypoint at rest
%    x_T(t_T) = p_T
%    x^(1)_T(t_T) = 0
%    x^(2)_T(t_T) = 0
%    x^(3)_T(t_T) = 0

k = numSegments;
A(idx:idx+3, 8*(k-1)+1:8*k) = [polyVect(ts(k+1),0);...
                                polyVect(ts(k+1),1); 
                                polyVect(ts(k+1),2);
                                polyVect(ts(k+1),3)];
b(idx,:) = path(k+1,:); 

% Finally solve system of equations to find trajectory coefficients for
% each of the given coordinate basis
for i = 1:xyz
    X(:,i) = A(:,:)\b(:,i);
end

end

% Sub-function to calculate polynomial array of desired derivative order
function vect = polyVect(t, derivativeOrder)
switch derivativeOrder
    % Position
    case 0
        vect = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1];
    case 1
        vect = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
    case 2
        vect = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0];
    case 3
        vect = [210*t^4, 120*t^3, 60*t^2, 24*t, 6, 0, 0, 0];
    case 4
        vect = [840*t^3, 360*t^2, 120*t, 24, 0, 0, 0, 0];
    case 5
        vect = [2520*t^2, 720*t, 120, 0, 0, 0, 0, 0];
    case 6
        vect = [5040*t, 720, 0, 0, 0, 0, 0, 0];
end
end