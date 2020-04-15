clc
clear all;
% %% activation on Desktop Lab
% addpath('F:\[GitHub]\Hand_deformation_model\functions');
% addpath('F:\[GitHub]\Hand_deformation_model\external');
% addpath('F:\[GitHub]\Hand_deformation_model\data');
%% activation on Mac 
addpath('/Users/user/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/user/Documents/GitHub/Hand_deformation_model/external');
addpath('/Users/user/Documents/GitHub/Hand_deformation_model/data');
%%
[V, F, FB, H] = function_loading_ply_file('S01_hand data.ply');
LMs = function_get_LM_from_iges('S01_LM8.igs');

scan_points = V;

load('sg_mesh.mat');

LMt = function_get_LM_from_iges('Template_LM8.igs');
Template_LM = zeros(8,1);
vertex_normals = zeros(size(V,1),3);
Idx = [1:size(V,1)]';

%%
% size of template vertices = n, % size of scan points = m
n = size(sg_mesh.vertices, 1); % group A supply = 6457
m = size(scan_points, 1); % group B demand = 6984                                    
pairs = zeros(n, 2);

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

%% rough fitting (using ABSOR) 
% things to do: add wrist crease LMs

[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
Vt = sg_mesh.vertices;
Vt(:,4) = 1;
Vt = Vt';

%regParams.M*Vt(:,1)
Vt_tf = Vt;

Vt_tf = regParams.M*Vt_tf;
Vt_tf(4,:) = [];
Vt_tf = Vt_tf';

keep = ismember(sg_mesh.assignment, [1:6]);
[vertices, faces] = filter_vertices(sg_mesh.vertices, sg_mesh.faces, keep);
[vertices_tf,faces_tf] = filter_vertices(Vt_tf, sg_mesh.faces, keep);

vertices_idx = Idx(keep);
normals = sg_mesh.normals;
normals = normals(keep, :);

% normal of scan points
for vertexIdx = 1:size(scan_points,1);
    F2 = F;
    LI = F2 == vertexIdx;
    [row2, col2] = find(LI);
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
    normals = getNormals(V, F);
    normals_F = normals(row2,:);
    normals_F = sum(normals_F);
    normals_F = normals_F/norm(normals_F);
    vni = normals_F; % weighted normal vector of vi
    vertex_normals(vertexIdx,:) = vni;
end 
%% Sub-sampling palm area
% centroid_t = [mean(vertices(:,1)),mean(vertices(:,2)),mean(vertices(:,3))];
% n = size(vertices,1);
% delta = vertices - repmat(centroid_t(1, :), n, 1);
% distances = sum(delta .^ 2, 2);
% [~, j] = max(distances);
% threshold_palm_cut = norm(centroid_t-vertices(j,:));
% 
% % centroid_t (threshold) hand scan points logical
% delta_p = scan_points - repmat(centroid_t(1, :), m, 1);
% distances_p = sqrt(sum(delta_p .^ 2, 2));
% distances_p(:,2) = [1:m]';
% 
% LIX = distances_p(:,1) < 120; % to do: how to determine threshold? 95 is minimum numbers that set boundary to cover vertices more than 1374(n)
% scan_points = scan_points(LIX);
% 
% % subsample scan points % index cutting template palm segments sub-sampling
% nPointsSample = min(n,size(scan_points,1));
% fprintf('nPointsSample: %d/%d\n',nPointsSample,size(scan_points,1));
% pointsIdxsScan = randperm(size(scan_points,1),nPointsSample); 
% scan_points_subsampled = V(pointsIdxsScan(1:nPointsSample),:);

%% template palm fitting to raw hand scan

n = size(vertices_tf,1); % # of palm = 1374
m = size(scan_points,1); % # of original scan = 6457
dummysize = m - n; % 5083

A = zeros(m,m); % m x m matrix 
% A = point to point distnace between template and scan points

for i = 1:n
    for j = 1:m
    cost(i,j) = norm(vertices_tf(i,:)-scan_points(j,:));
    end
end
cost(n+1:m,:) = 0;

supply = [1; 1; 1];
demand = [1 1 1 1 1 1];

%% ���� ����Ͽ� matching - transportation algorithm 
% �� �� transportation problem solving algorithm�� �����Ͽ� pair ���� testing

vertexIdx = 1;

vni = normals_F;
dv = av - dot(av,bv)/norm(bv)^2 * bv; 
cosTH = dot(dv,vni)/(norm(dv)*norm(vni));



%% visualization 
figure()
axis equal
axis off
hold on
scatter3(Vt_tf(:,1),Vt_tf(:,2),Vt_tf(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255]);
%scatter3(sg_mesh.vertices(:,1),sg_mesh.vertices(:,2),sg_mesh.vertices(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(scan_points(:,1),scan_points(:,2),scan_points(:,3),'.', 'MarkerEdgeColor',[0/255, 200/255, 0/255]);
hold off

figure()
axis equal
axis off
hold on
scatter3(vertices_tf(:,1),vertices_tf(:,2), vertices_tf(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(scan_points(:,1),scan_points(:,2),scan_points(:,3),'.', 'MarkerEdgeColor',[0/255, 200/255, 0/255]);
hold off


















