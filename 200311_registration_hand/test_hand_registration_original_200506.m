
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
normals = -per_vertex_normals(vertices, faces);
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

%%
O = vertices;
TF = faces;
TG = points.vertices;

figure()
axis equal
axis off
hold on
% original T-points = Gray color
scatter3(O(:,1),O(:,2),O(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
% target scan points = Light blue color
scatter3(TG(:,1),TG(:,2),TG(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]);
quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);
% Interest (palm segments) = Red color
% scatter3(IT(:,1),IT(:,2),IT(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

clear O TF TG

%% Keep only the palm and find rigid transform iteratively
keep = ismember(mesh.assignment, [1:6]);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);

distance_threshold = 50;

    n = size(vertices, 1);
    m = size(points.vertices, 1);
    pairs = zeros(n, 2);
    
    for i = 1 : n
        delta = points.vertices - repmat(vertices(i, :), m, 1);
        distances = sqrt(sum(delta .^ 2, 2));
        [~, j] = min(distances);
        pairs(i, 1) = i;
        pairs(i, 2) = j;
    end


pairs = pairs(keep    
    
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals, distance_threshold);
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