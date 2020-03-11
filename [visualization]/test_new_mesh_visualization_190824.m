addpath(genpath('external'));
addpath 'functions';
addpath '190808_vertex_separation'

[vertices, faces, FaceB, Header] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers.mat')

figure(1)
    hold on
    h = trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
    scatter3(centers(:,1), centers(:,2), centers(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]);
    axis equal;
    view([-90, 0]);
    camlight;
    view([90, 0]);
    camlight;
    grid off;
    lighting gouraud;
    axis off;
    hold off