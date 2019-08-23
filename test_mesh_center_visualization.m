addpath(genpath('external'));
addpath 'functions';

mesh = load('mesh/neutral.mat');
mesh = mesh.mesh;

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = bone_axes(mesh.spheres);

centers = zeros(30,3);
for i = 1:30;
centers(i,:) = mesh.spheres{1,i}.center;
end 
C = centers;
root = centers(26, :) * 0.5 + centers(27, :) * 0.5;

itpts = [12; 11; 9; 8]; %interest point index of center 
% n_thumb = 20, 19, 17, 16
% n_middle = 12, 11, 9, 8

B = C(itpts,:); %interest point of axes

n_middle = -finger_normal(centers(12, :), centers(11, :), centers(9, :), centers(8, :));
axes{9} = build(centers(12, :), centers(11, :) - centers(12, :), n_middle);
axes{10} = build(centers(11, :), centers(10, :) - centers(11, :), n_middle);
axes{11} = build(centers(10, :), centers(9, :) - centers(10, :), n_middle);


%% visualization of axis  
axes_3 = axes{1,3};
axes_3(1:3,1:3) = axes_3(1:3,1:3) + axes_3(1:3,4);
axes_3_line = zeros(6,3);
axes_3_line(1,1:3) = axes_3(1,1:3);
axes_3_line(3,1:3) = axes_3(2,1:3);
axes_3_line(5,1:3) = axes_3(3,1:3);
axes_3_line(2,1:3) = axes_3(1,4);
axes_3_line(4,1:3) = axes_3(2,4);
axes_3_line(6,1:3) = axes_3(3,4);

figure(1)
hold on
scatter3(axes_3(1,1),axes_3(2,1),axes_3(3,1),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
plot3(axes_3_line(1:2,1),axes_3_line(3:4,1),axes_3_line(5:6,1),'-r')
scatter3(axes_3(1,2),axes_3(2,2),axes_3(3,2),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
plot3(axes_3_line(1:2,2),axes_3_line(3:4,2),axes_3_line(5:6,2),'-g')
scatter3(axes_3(1,3),axes_3(2,3),axes_3(3,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
plot3(axes_3_line(1:2,3),axes_3_line(3:4,3),axes_3_line(5:6,3),'-b')
scatter3(axes_3(1,4),axes_3(2,4),axes_3(3,4),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
%scatter3(0,0,0,'.','MarkerEdgeColor',[0, 255/255, 222/255]) % origin(0,0,0), colored cyan

axes_9 = axes{1,9};
axes_9(1:3,1:3) = axes_9(1:3,1:3) + axes_9(1:3,4);
axes_9_line = zeros(6,3);
axes_9_line(1,1:3) = axes_9(1,1:3);
axes_9_line(3,1:3) = axes_9(2,1:3);
axes_9_line(5,1:3) = axes_9(3,1:3);
axes_9_line(2,1:3) = axes_9(1,4);
axes_9_line(4,1:3) = axes_9(2,4);
axes_9_line(6,1:3) = axes_9(3,4);

figure(1)
hold on
scatter3(axes_9(1,1),axes_9(2,1),axes_9(3,1),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
plot3(axes_9_line(1:2,1),axes_9_line(3:4,1),axes_9_line(5:6,1),'-r')
scatter3(axes_9(1,2),axes_9(2,2),axes_9(3,2),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
plot3(axes_9_line(1:2,2),axes_9_line(3:4,2),axes_9_line(5:6,2),'-g')
scatter3(axes_9(1,3),axes_9(2,3),axes_9(3,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
plot3(axes_9_line(1:2,3),axes_9_line(3:4,3),axes_9_line(5:6,3),'-b')
scatter3(axes_9(1,4),axes_9(2,4),axes_9(3,4),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
%scatter3(0,0,0,'.','MarkerEdgeColor',[0, 255/255, 222/255]) % origin(0,0,0), colored cyan


h = trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
axis equal;
grid off;
lighting gouraud;
axis off;
hold off



%% Render mesh
hold on
h = trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
axis equal;
grid off;
lighting gouraud;
axis off;
scatter3(C(:,1), C(:,2), C(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]);
% scatter3(B(:,1), B(:,2), B(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]);
plot3(n_thumb(1:2,1), n_thumb(1:2,2), n_thumb(1:2,3), '-k')

hold off