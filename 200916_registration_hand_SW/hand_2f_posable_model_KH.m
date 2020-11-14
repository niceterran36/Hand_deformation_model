% hand 2-finger posable model demo code
clc
clear all
format shortG

%% register library - PC Home
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');
format shortG
%% reference functions

% [generation transformation matrix]
% transforms = cell(1, 8);
% for i = 1 : 8
%     transforms{i} = eye(4);
% end
% axes = compute_bone_axes_2f(centers_2f);

% [segment information]
% assignment 1 = palm
% assignment 2 = D2 proximal phalanx
% assignment 3 = D2 middle phalanx
% assignment 4 = D2 distal phalanx
% assignment 5 = D1 metacarpal
% assignment 6 = D1 proximal phalanx
% assignment 7 = D1 distal phalanx

% [joint CoR information]
% centers_2f(1,:) - % Wrist
% centers_2f(2,:) - % D1 CMC
% centers_2f(3,:) - % D1 MCP
% centers_2f(4,:) - % D1 IP
% centers_2f(5,:) - % D1 tip
% centers_2f(6,:) - % D2 MCP
% centers_2f(7,:) - % D2 PIP
% centers_2f(8,:) - % D2 DIP
% centers_2f(9,:) - % D2 tip
% centers_2f(10,:) - % D3 MCP

% [Bone structure]
% mesh_2f.bonestructure = [1 6; 6 7; 7 8; 8 9; 1 2; 2 3; 3 4; 4 5; 1 8];
% mesh_2f.bones{1,1}.parent = 0;
% mesh_2f.bones{1,2}.parent = 1;
% mesh_2f.bones{1,3}.parent = 2;
% mesh_2f.bones{1,4}.parent = 3;
% mesh_2f.bones{1,5}.parent = 1;
% mesh_2f.bones{1,6}.parent = 5;
% mesh_2f.bones{1,7}.parent = 6;
% mesh_2f.bones{1,8}.parent = 1;

%% Load data
load('hand_2fingers.mat');

% import 2-finger hand template model
h_vertices = mesh_2f.vertices;
h_faces = mesh_2f.faces;
h_normals = per_vertex_normals(h_vertices, h_faces);
assignment = mesh_2f.assignment;

% import scan data
points = {};
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('MJ_P03.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);

%% correspondence pair detection 

keep = ismember(assignment, 2);
[vertices, faces] = filter_vertices(h_vertices, h_faces, keep);
normals = h_normals(keep, :);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals,20,cos(60*pi/180));

%% posable hand model (digit 2)

% axes call
% transforms call

% [generation transformation matrix]
transforms = cell(1, 10);
for i = 1 : 10
    transforms{i} = eye(4);
end
transforms_ad = cell(1,1);
for i = 1 
    transforms_ad{i} = eye(4);
end 
axes = compute_bone_axes_2f(mesh_2f.centers);
angle = zeros(4,1);
%  degree information: 1.5 = 90 deg., 0.1 = 6 deg, 1/60 = 1 deg.

%% input joint angle for posture change

% Digit 2 flexion(+)/extension(-) 
angle(1) = 0/60;
angle(2) = 0/60;
angle(3) = 0/60;
% Digit 2 abduction(+)/adduction(-) 
angle(4) = 0/60; 

%% joint rotation by transform matrix

% MCP abduction/adduction motion
transforms_ad{1} = matrix_rotation( ... % D2 MCP -ab/ad
    angle(4), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{1}, axes{6}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{1}, axes{6}(1 : 3, 4)') ... % center
) * transforms{1};

% Flexion/extension of Digit 2 
transforms{2} = matrix_rotation( ... % D2 MCP
    angle(1), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{1}, axes{6}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{1}, axes{6}(1 : 3, 4)') ... % center
) * transforms_ad{1} * transforms{1};
transforms{3} = matrix_rotation( ... % D2 PIP
    angle(2), ...
    matrix_apply(transforms{2}, axes{7}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{2}, axes{7}(1 : 3, 4)') ...
) * transforms{2};
transforms{4} = matrix_rotation( ... % D2 DIP
    angle(3), ...
    matrix_apply(transforms{3}, axes{8}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{3}, axes{8}(1 : 3, 4)') ...
) * transforms{3};

%%
transformed = mesh_2f;
transformed = skin_dualquat(transformed, transforms);

figure()
hold on;
axis equal
axis off
h = trimesh(transformed.faces, transformed.vertices(:, 1), transformed.vertices(:, 2), transformed.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off;




