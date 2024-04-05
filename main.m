%% Intelligent distributed system project
% Pettene Mattia - Poggesi Gabriele

%% Initialization

clc;
close all;
clear;

set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',  22)
set(0,'DefaultLegendFontSize', 20)

addpath('functions/');


%% Map creation

map = imread('usgsImageryLayer.tif');

figure('Name','Real map')
imshow('usgsImageryLayer.tif', 'InitialMagnification', 'fit');

[x,y] = meshgrid(1:size(map,1), 1:size(map,2));
z = double(map(:,:,1));

% Plot using mesh
figure;
mesh(x,y,z);
colorbar;

% Plot using surf (best one)
figure;
surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', 'texturemap');
colorbar;

%% Test changing resolution (coarser mesh)

resolution = 200;

new_x = linspace(1, size(map, 2), resolution); 
new_y = linspace(1, size(map, 1), resolution); 
new_z = interp2(x, y, double(map(:,:,1)), new_x', new_y);

figure;
mesh(new_x, new_y, new_z);
colorbar;

