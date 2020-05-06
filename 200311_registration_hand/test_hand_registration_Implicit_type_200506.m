
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');

%%
[Vsp, F, FB, H] = function_loading_ply_file('S01_hand data.ply');
LMs = function_get_LM_from_iges('S01_LM8.igs');

load('sg_mesh.mat');

LMt = function_get_LM_from_iges('Template_LM8.igs');
Template_LM = zeros(8,1);
Idx = [1:size(Vsp,1)]';


%%
%quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
%quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);

%%
% size of template vertices = n, % size of scan points = m
n = size(sg_mesh.vertices, 1); % group A supply = 6457
m = size(Vsp, 1); % group B dema

for i=1:8
delta = sg_mesh.vertices - repmat(LMt(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
% template index based LMt info. update
for i = 1:8
LMt(i,:) = sg_mesh.vertices(Template_LM(i),:);
end 

LMt = LMt'; LMs = LMs';

clear j delta distances 

[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
Vt = sg_mesh.vertices;
Vt(:,4) = 1;
Vt = Vt';

%regParams.M*Vt(:,1)
Vt_tf = Vt;

Vt_tf = regParams.M*Vt_tf;
Vt_tf(4,:) = [];
Vt_tf = Vt_tf'; 

%%

% normal of scan points
for vertexIdx = 1:size(Vsp,1)
    F2 = F;
    LI = F2 == vertexIdx;
    [row2, ~] = find(LI);
    F2 = F2(row2,:);
    
    for i = 1:size(F2,1)*3
        v(i,1) = F2(i);
    end
    
    nv = vertexIdx;
    for i = 1:size(v,1)
        if v(i) ~= nv
           nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
        end
    end
    
    normals = getNormals(Vsp, F);
    normals_F = normals(row2,:);
    normals_F = sum(normals_F);
    normals_F = normals_F/norm(normals_F);
    Normal_sp(vertexIdx,:) = normals_F; % weighted normal vector of vi
end

clear row2 distances ErrorStats i j LI normals nv LMs LMt Template_LM v vertexIdx Vt

% normal of transformed template vertices
for vertexIdx = 1:size(Vt_tf,1)
    F2T = sg_mesh.faces;
    LI2 = F2T == vertexIdx;
    [row3, ~] = find(LI2);
    F2T = F2T(row3,:);
    
    for i = 1:size(F2T,1)*3
        v(i,1) = F2T(i);
    end
    
    nv = vertexIdx;
    for i = 1:size(v,1)
        if v(i) ~= nv
           nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
        end
    end
    
    normals = getNormals(Vt_tf, sg_mesh.faces);
    normals_F = normals(row3,:);
    normals_F = sum(normals_F);
    normals_F = normals_F/norm(normals_F);
    Normal_t(vertexIdx,:) = normals_F; % weighted normal vector of vi
end

clear row2 col2 distances ErrorStats i j LI normals nv LMs LMt Template_LM v vertexIdx Vt

Normal_t_palm = Normal_t(keep, :);

%%

% Keep only the palm and find rigid transform iteratively
keep = ismember(sg_mesh.assignment, [1:6]); %filtering palm segment points
[Vt_palm, Fs_palm] = filter_vertices(Vt_tf , sg_mesh.faces, keep); %template palm  points
[Vt_tf_palm,faces_tf_palm] = filter_vertices(Vt_tf, sg_mesh.faces, keep); %transformed template palm points
Vt_palm_idx = Idx(keep);

vertices =  Vt_palm;
faces = Fs_palm;
normals = Normal_t_palm;

points.vertices = Vsp;
points.faces = F;
points.normals = Normal_sp;

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end

transform = eye(4);

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