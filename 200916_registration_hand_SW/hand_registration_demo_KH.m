% hand_register_demo code
clc
clear all
format shortG

% register library - PC Home
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');
format shortG
clc

% register library & functions
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');
format shortG

% register library - Macbook

addpath(genpath('../external'));
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/external/registration');
format shortG

% load template & scan data
load('hy_mesh_n4.mat'); %template
[V_2f, F_2f, ~, ~] = function_loading_ply_file('hand_template_2fingers.ply');
LMs = function_get_LM_from_iges('LMs3.igs');
LMt = function_get_LM_from_iges('LMt.igs');
points = {};
% import problem
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('MJ_P03.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);

m_vertices = V_2f;
m_faces = F_2f;
m_centers_c = mesh.centers;
m = size(V_2f, 1);

% size of template vertices = n, % size of scan points = m
n = size(V_2f, 1); 
m = size(points.vertices, 1);
Template_LM = zeros(size(LMs,1),1);

for i=1:size(LMs,1)
delta = V_2f - repmat(LMt(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
% template index based LMt info. update
for i = 1:size(LMs,1)
LMt(i,:) = V_2f(Template_LM(i),:);
end 

LMt = LMt'; LMs = LMs';
clear j delta distances 

% apply transformation matrix
[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
m_vertices = apply_matrix(regParams.M, m_vertices, 1);
m_centers_c = apply_matrix(regParams.M, m_centers_c, 1);

centers_2f = zeros(10,3);
centers_2f(1,:) = m_centers_c(22,:); % Wrist
centers_2f(2,:) = m_centers_c(20,:); % D1 CMC
centers_2f(3,:) = m_centers_c(19,:); % D1 MCP
centers_2f(4,:) = m_centers_c(18,:); % D1 IP
centers_2f(5,:) = m_centers_c(17,:); % D1 tip
centers_2f(6,:) = m_centers_c(16,:); % D2 MCP
centers_2f(7,:) = m_centers_c(15,:); % D2 PIP
centers_2f(8,:) = m_centers_c(14,:); % D2 DIP
centers_2f(9,:) = m_centers_c(13,:); % D2 tip
centers_2f(10,:) = m_centers_c(12,:); % D3 MCP

% generation transformation matrix
transforms = cell(1, 8);
for i = 1 : 8
    transforms{i} = eye(4);
end
axes = compute_bone_axes_2f(centers_2f);
m_normals = per_vertex_normals(m_vertices, m_faces);

figure()
hold on;
view_angle = [173,2];
view(view_angle);
trimesh(m_faces, m_vertices(:, 1), m_vertices(:, 2), m_vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
%quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
%quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);
hold off;
view([-90,0]);
camlight;
view([90, 0]);
camlight;
view(view_angle);
axis equal;
grid off;
lighting gouraud;
axis off;
title('initial registration');

% assignment update
% m_vertices = 2f hand model
% mesh.vertices = full hand model
Vidx = [1:size(mesh.vertices,1)]';

% segment information
% assignment 1 = palm
% assignment 2 = D2 proximal phalanx
% assignment 3 = D2 middle phalanx
% assignment 4 = D2 distal phalanx
% assignment 5 = D1 metacarpal
% assignment 6 = D1 proximal phalanx
% assignment 7 = D1 distal phalanx

% assignmet for seg1 (palm)
keep = ismember(mesh.assignment, 1:5);
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);

n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);

% pair detection
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end

palm_idx = pairs(:,2);
assignment_2f = zeros(size(m_vertices,1),1);
assignment_2f(palm_idx) = 1;

% assignmet for seg2 (D2 proximal phalanx)
keep = ismember(mesh.assignment, 9);
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D2_idx = pairs(:,2);
assignment_2f(D2_idx) = 2;

% assignmet for seg3 (D2 middle phalanx)
keep = ismember(mesh.assignment, 10);
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D2_idx2 = pairs(:,2);
assignment_2f(D2_idx2) = 3;

% assignmet for seg4 (D2 distal phalanx)
keep = ismember(mesh.assignment, 11); % update
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D2_idx3 = pairs(:,2); % update
assignment_2f(D2_idx3) = 4; % update

% assignmet for seg5 (D1 metacarpal)
keep = ismember(mesh.assignment, 6); % update
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D1_idx1 = pairs(:,2); % update
assignment_2f(D1_idx1) = 5; % update

% assignmet for seg6 (D1 proximal phalanx)
keep = ismember(mesh.assignment, 7); % update
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D1_idx2 = pairs(:,2); % update
assignment_2f(D1_idx2) = 6; % update

% assignmet for seg7 (D1 distal phalanx)
keep = ismember(mesh.assignment, 8); % update
vertices = mesh.vertices;
faces = mesh.faces;
[vertices, faces] = filter_vertices(vertices, faces, keep);
n = size(V_2f, 1); 
m = size(vertices, 1);
pairs = zeros(m,2);
pairs(:,1) = Vidx(keep);
for i=1:size(vertices,1)
delta = V_2f - repmat(vertices(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
pairs(i,2) = j;
end
D1_idx3 = pairs(:,2); % update
assignment_2f(D1_idx3) = 7; % update


% palm registration
keep = ismember(assignment_2f, 1);
[vertices, faces] = filter_vertices(m_vertices, m_faces, keep);
normals = m_normals(keep, :);
pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
figure()
view(view_angle);
transform = eye(4);
for i = 1 : 5
    delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
    transform = delta * transform;
    vertices = apply_matrix(delta, vertices);
    normals = apply_matrix(delta, normals, 0);
    pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
    v = get(gca, 'view'); 
    trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.1);
    hold on;
    trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1);
    plot3( ...
        [vertices(pairs(:, 1), 1), points.vertices(pairs(:, 2), 1)]', ...
        [vertices(pairs(:, 1), 2), points.vertices(pairs(:, 2), 2)]', ...
        [vertices(pairs(:, 1), 3), points.vertices(pairs(:, 2), 3)]', ...
    'Color', 'red');
    hold off;
    view([-90,0]);
    camlight;
    view([90,0]);
    camlight;
    axis equal;
    grid off;
    lighting gouraud;
    axis off;
    title(['After ', num2str(i), ' rigid transformation']);
    set(gca, 'view', v);
    pause(0.01);
end

transforms{2} = transform;

m_vertices = apply_matrix(transform, m_vertices, 1); % update current vertices
m_centers_2f = apply_matrix(transform, centers_2f); % update current centers
m_normals = per_vertex_normals(m_vertices, m_faces);

mesh_2f.vertices = m_vertices;
mesh_2f.faces = m_faces;
mesh_2f.assignment = assignment_2f;
mesh_2f.normals = m_normals;
mesh_2f.centers = m_centers_2f;
mesh_2f.bonestructure = [1 6; 6 7; 7 8; 8 9; 1 2; 2 3; 3 4; 4 5; 1 10];
mesh_2f.bones{1,1}.parent = 0;
mesh_2f.bones{1,2}.parent = 1;
mesh_2f.bones{1,3}.parent = 2;
mesh_2f.bones{1,4}.parent = 3;
mesh_2f.bones{1,5}.parent = 0;
mesh_2f.bones{1,6}.parent = 5;
mesh_2f.bones{1,7}.parent = 6;
mesh_2f.bones{1,8}.parent = 7;
mesh_2f.bones{1,9}.parent = 0;
mesh_2f.bones{1,10}.parent = 0;

sphere_bone = [9 5 6 7 8 1 2 3 4 9];

for i  = 1:10
   mesh_2f.spheres{1,i}.center = m_centers_2f(i,:);
   mesh_2f.spheres{1,i}.bone = sphere_bone(i);
end

save hand_2fingers.mat mesh_2f % hand_2fingers.mat <-- save 2 finger hand template upto here.

% check for segmentation
V = m_vertices(D1_idx3,:);
C = m_centers_2f;

figure()
hold on
axis equal
axis off
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(C(:,1),C(:,2),C(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

