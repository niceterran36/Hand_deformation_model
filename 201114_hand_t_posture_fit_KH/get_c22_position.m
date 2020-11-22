clc
clear all
close all

addpath('C:\Users\user\Desktop\SHu');
addpath('angle data\');
addpath('Data\');
addpath("Fitted vertices\");
addpath('D:\GitHub\Hand_deformation_model\data_SW\');
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\external\registration\');
addpath('D:\GitHub\Hand_deformation_model\200916_registration_hand_SW\');

dirLM = dir('C:\Users\user\Desktop\SHu\*.igs');
dir3D = dir('C:\Users\user\Desktop\SHu\*.ply');

centers22 = zeros(size(dir3D,1),3);

    
for ii = 1:9
    
    load('hy_mesh_n5.mat'); %template
    
    [points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file(dir3D(ii).name);
    points.normals = per_vertex_normals(points.vertices, points.faces);
    fprintf('%d, %s\n', ii, dir3D(ii).name);
    %LMt = function_get_LM_from_iges(dirLM(ii).name);
     
    % palm scale 
    % Landmarks for palm alignment & hand scale
    LMs_PLM = function_get_LM_from_iges(dirLM(ii).name); % LM for scan
    LMt_PLM = function_get_LM_from_iges('LMt_PLM.igs'); % LM for template
    LMs_PLM = LMs_PLM'; LMt_PLM = LMt_PLM';
    [regParams,~,~] = absor(LMt_PLM,LMs_PLM);

    % update vertices & CoR
    mesh.vertices = apply_matrix(regParams.M, mesh.vertices, 1);
    for i = 1:22
        mesh.spheres{1,i}.center = apply_matrix(regParams.M, mesh.spheres{1,i}.center, 1);
        mesh.centers(i,:) = mesh.spheres{1,i}.center;
    end

% palm registration
keep = ismember(mesh.assignment, 1:5);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
normals = per_vertex_normals(mesh.vertices, mesh.faces);
normals = normals(keep, :);
pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
view_angle = [207,10];

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

    % update vertices & CoR
    mesh.vertices = apply_matrix(transform, mesh.vertices, 1); % update current vertices
    mesh.normals = per_vertex_normals(mesh.vertices, faces);
    for i = 1:22
        mesh.spheres{1,i}.center = apply_matrix(transform, mesh.spheres{1,i}.center,1);
        mesh.centers(i,:) = mesh.spheres{1,i}.center;
    end

    clear Bfit ErrorStats regParams delta faces h i keep LMs_PLM LMt_PLM pairs transform transforms

    centers22(ii,:) = mesh.spheres{1,22}.center
    clear mesh
end 

    
    