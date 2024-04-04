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

% 46 00 22 N    11 01 15 E  -----> lat 46.006111  lon 11.020833
% 46 00 22 N    11 02 08 E  -----> lat 46.006111  lon 11.035556
% 46 01 14 N    11 02 08 E  -----> lat 46.020556  lon 11.035556
% 46 01 14 N    11 01 15 E  -----> lat 46.020556  lon 11.020556


% latlim = [46.006111 46.020556];
% lonlim = [11.020833 11.035556];
% rasterSize = [100 100];
% R = georefpostings(latlim,lonlim,rasterSize);
% N = egm96geoid(R);
% 
% 
% 
% figure
% axesm miller
% axis off
% 
% geoshow(Z,R,"DisplayType","surface")
% 
% contour3m(Z,R,40,"k")
% 
% daspect([1 1 40])
% view(3)


useInternet = true;

bmapLayers = wmsfind('elevation','SearchFields','layername');
usgsImageryLayer = refine(bmapLayers,"GreyHillshade","SearchFields","layername");

if useInternet
    usgsImageryLayer = wmsupdate(usgsImageryLayer);
    save("usgsImageryLayer.mat","usgsImageryLayer")
else
    load usgsImageryLayer.mat
end

%latlim = [42.3453 42.3711];
%lonlim = [-71.099 -71.0454];
latlim = [46.006111 46.020556];
lonlim = [11.020833 11.035556];
s = 750;

if useInternet
    [A,R] = wmsread(usgsImageryLayer,"Latlim",latlim,"Lonlim",lonlim,"ImageHeight",s,"ImageWidth",s);
    geotiffwrite("Viote0.tif",A,R)
else
    [A,R] = readgeoraster("Viote0.tif");
end

figure
usamap(A1,R)
geoshow(A1,R)



