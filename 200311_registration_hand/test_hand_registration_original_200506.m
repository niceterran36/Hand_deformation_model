
% Register gptoolbox
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');

% Load neutral mesh and point cloud

load('hy_mesh_n.mat');
points = {};
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('S01_hand data.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);
transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = compute_bone_axes(mesh.spheres);
%% 
transform = inv(initial_guess(points.vertices)) * initial_guess(mesh.vertices);
faces = mesh.faces;
vertices = apply_matrix(transform, mesh.vertices);
normals = per_vertex_normals(vertices, faces);
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

%% Keep only the palm and find rigid transform iteratively
keep = ismember(mesh.assignment, [1:6]);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);

mesh_vertices = vertices;
mesh_normals = normals;
points_vertices = points.vertices;
points_normals = points.normals;
distance_threshold = 30;
cos_angle_threshold = 0.5;


pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
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
% TODO improve proto-phalange?
transforms{15} = transform;
%pause;