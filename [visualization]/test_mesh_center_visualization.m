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

figure(1) % axis{3} plotting
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

figure(1) % axis{9} plotting
hold on
scatter3(axes_9(1,1),axes_9(2,1),axes_9(3,1),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
plot3(axes_9_line(1:2,1),axes_9_line(3:4,1),axes_9_line(5:6,1),'-r')
scatter3(axes_9(1,2),axes_9(2,2),axes_9(3,2),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
plot3(axes_9_line(1:2,2),axes_9_line(3:4,2),axes_9_line(5:6,2),'-g')
scatter3(axes_9(1,3),axes_9(2,3),axes_9(3,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
plot3(axes_9_line(1:2,3),axes_9_line(3:4,3),axes_9_line(5:6,3),'-b')
scatter3(axes_9(1,4),axes_9(2,4),axes_9(3,4),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
%scatter3(0,0,0,'.','MarkerEdgeColor',[0, 255/255, 222/255]) % origin(0,0,0), colored cyan
hold off

%% visualization of axis with 'for loop'

% translate the axes vector from origin to each axis center 
axes_t = axes;
for i = 1:18;
axes_t{1,i}(1:3,1:3) = axes_t{1,i}(1:3,1:3)+axes_t{1,i}(1:3,4);
end    

% variables of axis end-point & center-point 
axes_x_pt = zeros(18,3); axes_y_pt = zeros(18,3); axes_z_pt = zeros(18,3); axes_center_pt = zeros(18,3);
    for i = 1:18
    axes_x_pt(i,:) = axes_t{1,i}(1:3)';
end
for i = 1:18
    axes_y_pt(i,:) = axes_t{1,i}(5:7)';
end
for i = 1:18
    axes_z_pt(i,:) = axes_t{1,i}(9:11)';
end
for i = 1:18
    axes_center_pt(i,:) = axes_t{1,i}(13:15)';
end  

figure (1)
hold on
axis equal
axis off
grid off
% line plotting format plot3([A(1) B(1)],[A(2) B(2)],[A(3) B(3)], '-k')

    for i = 1:18 % x-axis line drawing with red
        plot3([axes_t{1,i}(1,1) axes_t{1,i}(1,4)],[axes_t{1,i}(2,1) axes_t{1,i}(2,4)],[axes_t{1,i}(3,1) axes_t{1,i}(3,4)], '-r')
    end 
    for i = 1:18 % y-axis line drawing with green
        plot3([axes_t{1,i}(1,2) axes_t{1,i}(1,4)],[axes_t{1,i}(2,2) axes_t{1,i}(2,4)],[axes_t{1,i}(3,2) axes_t{1,i}(3,4)], '-g')
    end 
    for i = 1:18 % z-axis line drawing with blue
        plot3([axes_t{1,i}(1,3) axes_t{1,i}(1,4)],[axes_t{1,i}(2,3) axes_t{1,i}(2,4)],[axes_t{1,i}(3,3) axes_t{1,i}(3,4)], '-b')
    end 
% axis end-point, center-point plotting
    scatter3(axes_x_pt(:,1),axes_x_pt(:,2),axes_x_pt(:,3),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
    scatter3(axes_y_pt(:,1),axes_y_pt(:,2),axes_y_pt(:,3),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
    scatter3(axes_z_pt(:,1),axes_z_pt(:,2),axes_z_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
    scatter3(axes_center_pt(:,1),axes_center_pt(:,2),axes_center_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
hold off    
    
    
    
    
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