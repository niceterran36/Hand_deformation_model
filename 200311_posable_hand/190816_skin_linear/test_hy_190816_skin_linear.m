% Register gptoolbox
addpath(genpath('external'));
addpath 'functions'

load('centers1.mat');
B = centers;

load('mesh_hy_altered.mat')
[vertices, faces, FaceB, Header] = function_loading_ply_file('hand_meshmodel_190730.ply');
A = vertices;

% for i = 1 : 30
%     mesh.spheres{1,i}.center(1,:) = B(i,:);
% end 

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
% axes = bone_axes(mesh.spheres);
axes = bone_axes(B);


transforms{6} = matrix_rotation(...
    0, ...
    matrix_apply(transforms{2}, axes{6}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ...
    ) * transforms{2};
transforms{7} = matrix_rotation( ...
    0, ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 4)') ...
) * transforms{6};
transforms{8} = matrix_rotation( ...
    0, ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 4)') ...
) * transforms{7};
transformed = mesh;
transformed = skin_linear(transformed, transforms);

vertices_T = transformed.vertices;

figure(1)
h = trimesh(faces, vertices_T(:, 1), vertices_T(:, 2), vertices_T(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
axis equal;
grid off;
lighting gouraud;
axis off;

figure(2)
h = trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
axis equal;
grid off;
lighting gouraud;
axis off;


============================================================================================

root = B(26, :) * 0.5 + B(27, :) * 0.5;
n_thumb = -finger_normal(B(20, :), B(19, :), B(17, :), B(16, :));
n100 = n_thumb*50;

n_index = -finger_normal(B(16, :), B(15, :), B(13, :), B(12, :));
ni100 = n_index*50;

ip = [16 15 13 12];
C = zeros(size(ip,2),3);
for i = 1:size(ip,2);
        C(i,:) = B(ip(i),:);
end

figure(2)
hold on
axis equal
axis off

scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[220/255, 211/255, 211/255]);
scatter3(B(:,1),B(:,2),B(:,3),'.', 'MarkerEdgeColor',[255/255, 0, 0]);  
scatter3(C(:,1),C(:,2),C(:,3),'.', 'MarkerEdgeColor',[0/255, 255/255, 30/255]); % interest point (ip)
scatter3(root(:,1),root(:,2),root(:,3),'.', 'MarkerEdgeColor',[240/255, 0/255, 255/255]); % root
% plot3(D(1:2,1),D(1:2,2),D(1:2,3), 'k-')

hold off