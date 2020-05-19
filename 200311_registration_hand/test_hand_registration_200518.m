%% register library - PC
% addpath(genpath('../external'));
% addpath('F:\[GitHub]\Hand_deformation_model\functions');
% addpath('F:\[GitHub]\Hand_deformation_model\data');
% addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
%% register library - Labtop
addpath(genpath('../external'));
addpath('C:\Users\EDT-jhy\Documents\GitHub\Hand_deformation_model\data');
addpath('C:\Users\EDT-jhy\Documents\GitHub\Hand_deformation_model\functions');
%% Load data_S0
load('hy_mesh_n3.mat');
LMs = function_get_LM_from_iges('LM_mjhand.igs');
LMt = function_get_LM_from_iges('Template_LM8.igs');
points = {};
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('mj_scanhand.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);

vertices = mesh.vertices;
faces = mesh.faces;
centers_c = mesh.centers;

%%
% size of template vertices = n, % size of scan points = m
n = size(mesh.vertices, 1); 
m = size(points.vertices, 1);
Template_LM = zeros(8,1);

for i=1:8
delta = mesh.vertices - repmat(LMt(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
% template index based LMt info. update
for i = 1:8
LMt(i,:) = mesh.vertices(Template_LM(i),:);
end 

LMt = LMt'; LMs = LMs';
clear j delta distances 

[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
vertices = apply_matrix(regParams.M, vertices, 1);
centers_c = apply_matrix(regParams.M, centers_c, 1);

%% 
transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = compute_bone_axes(mesh.spheres);

normals = per_vertex_normals(vertices, faces);

figure()
trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
hold on;
trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);
hold off;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
axis equal;
grid off;
lighting gouraud;
axis off;
title('Initial guess');
% pause;

vertices_c = vertices;
faces_c = faces;

%% palm registration
keep = ismember(mesh.assignment, 1:5);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
figure()
transform = eye(4);
for i = 1 : 10
    delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
    transform = delta * transform;
    vertices = apply_matrix(delta, vertices);
    normals = apply_matrix(delta, normals, 0);
    pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
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
    view([-90, 0]);
    camlight;
    view([90, 0]);
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

vertices_c = apply_matrix(transform, vertices_c, 1); % update current vertices
centers_c = apply_matrix(transform, centers_c); % update current centers
normals = per_vertex_normals(vertices_c, faces);

vertices_b = vertices_c; % vertices_b = vertices after palm registration
centers_b = centers_c; % centers_b = centers after palm registration

hand_template = mesh;
for i = 1:18
    hand_template.spheres{1,i}.center = centers_c(i,:);
end
hand_template.vertices = vertices_c;
hand_template.normals = per_vertex_normals(hand_template.vertices, hand_template.faces);

clear Bfit ErrorStats LMs LMt regParams Template_LM

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off;

%% parameter for registration order

P_segment = [6:20];
P_cor = [20 19 18 16 15 14 12 11 10 8 7 6 4 3 2];
P_digits{1} = [6:8];
P_digits{2} = [7:8];
P_digits{3} = 8;
P_digits{4} = [9:11];
P_digits{5} = [10:11];
P_digits{6} = 11;
P_digits{7} = [12:14];
P_digits{8} = [13:14];
P_digits{9} = 14;
P_digits{10} = [15:17];
P_digits{11} = [16:17];
P_digits{12} = 17;
P_digits{13} = [18:20];
P_digits{14} = [19:20];
P_digits{15} = 20;

P_cor_tr{1} = [17:19];
P_cor_tr{2} = [17:18];
P_cor_tr{3} = 17;
P_cor_tr{4} = [13:15];
P_cor_tr{5} = [13:14];
P_cor_tr{6} = 13;
P_cor_tr{7} = [9:11];
P_cor_tr{8} = [9:10];
P_cor_tr{9} = 9;
P_cor_tr{10} = [5:7];
P_cor_tr{11} = [5:6];
P_cor_tr{12} = 5;
P_cor_tr{13} = [1:3];
P_cor_tr{14} = [1:2];
P_cor_tr{15} = 1;

%% D1-D5 MCP,PIP,DID registration
h3 = [];
h4 = [];

for j = 1:15

vertices = vertices_c;
faces = faces_c;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, P_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);

transform = eye(4);

figure(2)
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, centers_c(P_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
        v = get(gca, 'view');
        trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.1);
        hold on;
        h3 = trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1);
        h4 = plot3( ...
            [vertices(pairs(:, 1), 1), points.vertices(pairs(:, 2), 1)]', ...
            [vertices(pairs(:, 1), 2), points.vertices(pairs(:, 2), 2)]', ...
            [vertices(pairs(:, 1), 3), points.vertices(pairs(:, 2), 3)]', ...
        'Color', 'red');
        hold off;
        view([-90, 0]);
        camlight;
        view([90, 0]);
        camlight;
        axis equal;
        grid off;
        lighting gouraud;
        axis off;
        title(['After ', num2str(i), ' rigid transformation']);
        set(gca, 'view', v);
        pause(0.01);
    end

transforms{j+2} = transform;
   
keep = ismember(mesh.assignment, P_digits{j});
vi_Dx = vertices_c(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

vertices_c(keep,:) = vi_Dx;
centers_c(P_cor_tr{j},:) = apply_matrix(transform, centers_c(P_cor_tr{j},:), 1);
normals = per_vertex_normals(vertices_c, faces);

end

figure(3)
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% 
% D1 CMC
transforms{3} = transforms{3};
% D1 MCP
transforms{4} = transforms{4} * transforms{3};
% D1 IP
transforms{5} = transforms{5} * transforms{4};
% D2 MCP
transforms{6} = transforms{6};
% D2 PIP
transforms{7} = transforms{7} * transforms{6};
% D2 DIP
transforms{8} = transforms{8} * transforms{7};
% D3 MCP
transforms{9} = transforms{9};
% D3 PIP
transforms{10} = transforms{10} * transforms{9};
% D3 DIP
transforms{11} = transforms{11} * transforms{10};
% D4 MCP
transforms{12} = transforms{12};
% D4 PIP
transforms{13} = transforms{13} * transforms{12};
% D4 DIP
transforms{14} = transforms{14}  * transforms{13};
% D5 MCP
transforms{16} = transforms{16};
% D5 PIP
transforms{17} = transforms{17} * transforms{16};
% D5 DIP
transforms{18} = transforms{18} * transforms{17};


%% DQS application & Result display

hand_template = skin_dualquat(hand_template, transforms);

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off;

figure()
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% Things to do

% Apply dual quarternion skinning to root --> pose fitting --> DQS -->
% iteration

%% ICP registration

targetV = points.vertices;
sourceV = hand_template.vertices;
targetF = points.faces;
sourceF = hand_template.faces;
iterations = 30;
flag_prealligndata = 1;
figureOn = 1;
rigidICP = 0;

[sourceV] = ICP_nonrigidICP(targetV, sourceV, targetF, sourceF, iterations, flag_prealligndata, figureOn, rigidICP)

vertices_c = sourceV;
% clear targetV sourceV targetF sourceF 

figure(3)
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% memo - DQS required variable


transforms_ad{1} = matrix_rotation( ... % D2 MCP -ab/ad
    angle(16), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms{2};



transformed = skin_dualquat(transformed, transforms);

bones.parent
mesh.spheres{i}.center
mesh.vertices = dualquatlbs(mesh.vertices, DQ, mesh.weights);
mesh.normals = dualquatlbs(mesh.normals, DQ_normals, mesh.weights);


for i = 1:18
    mesh.spheres{1,i}.center = centers_c(i,:);
end 


















