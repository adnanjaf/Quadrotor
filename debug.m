% Dummy script for debugging
clear all;
close all;
addpath(genpath('./'));

%% My Map
% map = load_map('mymap.txt', 0.5, 1.0, 0.25);
% start = [7.0,-4.0,2.5];
% stop = [2.0,20.0,2.5];
% path = dijkstra(map, start, stop, true);

%% Map 0
% map = load_map('map0.txt', 0.5, 0.5, 0.25);
% start = [0,2,5];
% stop = [19,3,5];
% path = dijkstra(map, start, stop, true);

%% Map 1
% map = load_map('map1.txt', 0.2, 0.5, 0.25);
% start = [0.0, -4.9, 0.2];
% stop = [8.0, 18.0, 3.0];
% path = dijkstra(map, start, stop, true);

%% Map 2
% map = load_map('map2.txt', 0.2, 0.25, 0.25);
% start = [2, -3,0];
% stop = [5, 25,4];
% path = dijkstra(map, start, stop, true);

%% LOAD EXISTING PATHS
% IF YOU WISH TO SKIP THE PATH-PLANNING STAGE, UNCOMMENT ONE OF THE

% My Map
% testData = load('testPathMyMap_xyres0.5.mat');

% Map 0
% testData = load('testPath0.mat');

% Map 1
% testData = load('testPathMap1_xyres0.2.mat');

% Map 2
% testData = load('testPathMap2_xyres0.2_zres0.5.mat');

path = testData.path;
map = testData.map;
start = testData.start;
stop = testData.stop;

trajectory_generator([], [], map, path);
trajectory = test_trajectory(start, stop, map, path, true);
