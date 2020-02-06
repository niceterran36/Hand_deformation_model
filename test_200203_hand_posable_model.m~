clc; clear all;
addpath functions;
addpath(genpath('external'));
load('sg_mesh.mat');
centers = sg_mesh.centers; v_segment = sg_mesh.assignment; V = sg_mesh.vertices; F = sg_mesh.faces;
S = sg_mesh.bonestructure;

%% visualization
A = V;
figure()
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[16/255, 241/255, 255/255])
hold off

