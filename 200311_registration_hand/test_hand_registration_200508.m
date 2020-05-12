%% register library - PC
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
%% register library - Labtop
addpath(genpath('../external'));
addpath('C:\Users\EDT-jhy\Documents\GitHub\Hand_deformation_model\data');
addpath('C:\Users\EDT-jhy\Documents\GitHub\Hand_deformation_model\functions');
%% Load data
load('hy_mesh_n.mat');
LMs = function_get_LM_from_iges('LM_mjhand.igs');
LMt = function_get_LM_from_iges('Template_LM8.igs');
points = {};
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('mj_scanhand.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);
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
Vt = mesh.vertices;
Ct = mesh.centers;
Vt = apply_matrix(regParams.M, Vt, 1);
Ct = apply_matrix(regParams.M, Ct, 1);
vertices = Vt;
%% 
transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = compute_bone_axes(mesh.spheres);

faces = mesh.faces;
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

vertices_b = vertices;
faces_b = faces;

%% palm registration
keep = ismember(mesh.assignment, [1:5]);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);

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

%% 
vertices = vertices_b;
faces = faces_b;

vertices = apply_matrix(transforms{1,2}, vertices, 1);
normals = per_vertex_normals(vertices, faces);

vertices_b = vertices;
faces_b = faces;

%% D1 Metacarpals (root) registration
% == start here == thumb finger segmentation update needed

vertices = vertices_b;
faces = faces_b;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, 6); % digit1 root
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);

    for i = 1 : 10
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, Ct(20,:));
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
    
transforms{3} = transform;
       
%%

vertices = vertices_b;
faces = faces_b;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, [6:8]); 
[vertices_D1, faces_D1] = filter_vertices(vertices, faces, keep);

vertices_D1 = apply_matrix(transforms{1,3}, vertices_D1, 1);
normals_D1 = per_vertex_normals(vertices_D1, faces_D1);


%% D1 Proximal Phalanx registration

vertices = vertices_b;
faces = faces_b;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, 7);
[vertices, faces] = filter_vertices(vertices, faces, keep);
vertices = apply_matrix(transforms{1,3}, vertices, 1);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);

    for i = 1 : 10
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, Ct(19,:));
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
    
transforms{4} = transform;
    

%%
O = vertices;
TG = points.vertices;

figure()
axis equal
axis off
hold on
% original T-points = Gray color
scatter3(O(:,1),O(:,2),O(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(Ct(19,1),Ct(19,2),Ct(19,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
% transformed T-points = Light blue color
%scatter3(TF(:,1),TF(:,2),TF(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]); 
% target scan points = Green color
scatter3(TG(:,1),TG(:,2),TG(:,3),'.', 'MarkerEdgeColor',[143/255, 230/255, 143/255]);
% Interest (palm segments) = Red color
%scatter3(IT(:,1),IT(:,2),IT(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off


%%
figure()
trimesh(faces, O(:, 1), O(:, 2), O(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
hold on;
trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
quiver3(O(:, 1), O(:, 2), O(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
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

