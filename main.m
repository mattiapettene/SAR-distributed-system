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
addpath('Plot/');


%% Map creation

map = load("Rifugio_Viote.txt");

s = sqrt(length(map));
H = zeros(s);

for y_i = 1:s
    for x_i = 1:s
        idx = x_i + (y_i-1)*1000;
        H(x_i,y_i) = map(idx, 3);
    end
end

H = flipud(H);

figure('Name','3D map')
hold on;
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
saveas(gcf, 'Plot/3D_map.eps', 'epsc')