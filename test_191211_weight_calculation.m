clc
clear all;
addpath functions;
addpath(genpath('external'));

% function, initial parameter setting
a = 1.3;
c = 0.25;
fx = a*exp(-((x-0.5)^2)/(2*c^2));

W = zeros(size(V,1),21);

vertexIdx = 1;

% main influence joint = jna of assigned segment