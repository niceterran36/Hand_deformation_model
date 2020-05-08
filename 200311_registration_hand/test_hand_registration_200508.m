
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');

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
Vt(:,4) = 1;
Vt = Vt';

%regParams.M*Vt(:,1)
Vt_tf = Vt;

Vt_tf = regParams.M*Vt_tf;
Vt_tf(4,:) = [];
Vt_tf = Vt_tf'; 

vertices = Vt_tf;
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
pause;

keep = ismember(mesh.assignment, [1:6]);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
%%

transform = eye(4);
for i = 1 : 20
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

keep = mesh.assignment == 9; % digit2 root
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
normals = mesh.normals(keep, :);
vertices = apply_matrix(transform, vertices);
normals = apply_matrix(transform, normals, 0);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals, 0.1, 0.2);

    for i = 1 : 10
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, apply_matrix(transforms{2}, mesh.centers(16,:)));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        pairs = compute_correspondences(vertices, normals, points.vertices, points.normals, 0.3, 0.3);
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
    transforms{9} = transform;

%%
O = vertices;
TG = points.vertices;

figure()
axis equal
axis off
hold on
% original T-points = Gray color
scatter3(O(:,1),O(:,2),O(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
% transformed T-points = Light blue color
%scatter3(TF(:,1),TF(:,2),TF(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]); 
% target scan points = Green color
scatter3(TG(:,1),TG(:,2),TG(:,3),'.', 'MarkerEdgeColor',[143/255, 230/255, 143/255]);
% Interest (palm segments) = Red color
%scatter3(IT(:,1),IT(:,2),IT(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off
