% hand_register_demo code

% register library & functions
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');

% register library - Macbook
clc
clear all
addpath(genpath('../external'));
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/external/registration');
format shortG

% load template & scan data

load('hy_mesh_n4.mat'); %template
[V_2f, F_2f, FB_2f, H_2f] = function_loading_ply_file('hand_template_2fingers.ply');
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

% generation transformation matrix
transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = compute_bone_axes(mesh.spheres);
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

% palm registration
keep = ismember(mesh.assignment, 1:5);
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

m_vertices_c = apply_matrix(transform, m_vertices, 1); % update current vertices
m_centers_c = apply_matrix(transform, m_centers_c); % update current centers
m_normals = per_vertex_normals(m_vertices_c, m_faces);

hand_template = mesh;
for i = 1:18
    hand_template.spheres{1,i}.center = m_centers_c(i,:);
end
hand_template.vertices = m_vertices_c;
hand_template.normals = per_vertex_normals(hand_template.vertices, hand_template.faces);

clear Bfit ErrorStats LMs LMt regParams Template_LM

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([193, -4]);
camlight;
view([185, 8]);
camlight;
hold off;

save hand_template.mat hand_template

% parameter for finger root (MCP) registration 

load('hand_template.mat')
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('MJ_P03.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);

FRP_segment = [9];
FRP_cor = [16];
%FRP_digits{1} = [6:8];
FRP_digits{1} = [9:11];
%FRP_digits{3} = [12:14];
%FRP_digits{4} = [15:17];
%FRP_digits{5} = [18:20];

%FRP_cor_tr{1} = [17:19];
FRP_cor_tr{1} = [13:15];
%FRP_cor_tr{3} = [9:11];
%FRP_cor_tr{4} = [5:7];
%FRP_cor_tr{5} = [1:3];

transform_order= [6];

view_angle = [173,2];

% D1-D5 finger root (MCP) registration
h3 = [];
h4 = [];

for j = 1

vertices = hand_template.vertices;
faces = hand_template.faces;
normals = per_vertex_normals(vertices, faces);

keep = ismember(hand_template.assignment, FRP_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
%pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 25);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));


transform = eye(4);

figure(2)
    view(view_angle);
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, hand_template.spheres{1,j}.center);
        
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        
%        pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
        pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 25);

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

transforms{transform_order(j)} = transform;
   
keep = ismember(hand_template.assignment, FRP_digits{j});
vi_Dx = hand_template.vertices(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

% from here - recheck code

hand_template.vertices(keep,:) = vi_Dx;
hand_template.spheres{1,j}.center = apply_matrix(transform, hand_template.spheres{1,j}.center, 1);
hand_template.normals = per_vertex_normals(hand_template.vertices, hand_template.faces);

end

figure(3)
axis equal
axis off
hold on
view(view_angle);
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off