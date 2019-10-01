
[ListVertex, ListFace, ListFace_backup, HEADER] = function_loading_ply_file('hand_meshmodel_190730.ply');
A = ListVertex;
B = ListFace;

figure(1)
hold on
axis equal
axis off
h = trimesh(B, A(:, 1), A(:, 2), A(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 1);
        lighting gouraud;
    view([-90, 0]);
    camlight;
    view([90, 0]);
    camlight;
hold off

