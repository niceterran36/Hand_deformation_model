clc
clear all;
%% activation on Desktop Lab
 addpath('F:\[GitHub]\Hand_deformation_model\functions');
 addpath('F:\[GitHub]\Hand_deformation_model\external');
 addpath('F:\[GitHub]\Hand_deformation_model\data');
%% activation on Mac 
% addpath('/Users/user/Documents/GitHub/Hand_deformation_model/data');
% addpath('/Users/user/Documents/GitHub/Hand_deformation_model/external');
% addpath('/Users/user/Documents/GitHub/Hand_deformation_model/data');
%%
[Vsp, F, FB, H] = function_loading_ply_file('S01_hand data.ply');
LMs = function_get_LM_from_iges('S01_LM8.igs');

load('sg_mesh.mat');

LMt = function_get_LM_from_iges('Template_LM8.igs');
Template_LM = zeros(8,1);
Idx = [1:size(Vsp,1)]';

%%
% size of template vertices = n, % size of scan points = m
n = size(sg_mesh.vertices, 1); % group A supply = 6457
m = size(Vsp, 1); % group B demand = 6984                                    
%pairs = zeros(n, 2);

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

keep = ismember(sg_mesh.assignment, [1:6]); %filtering palm segment points
[Vt_palm, Fs_palm] = filter_vertices(sg_mesh.vertices, sg_mesh.faces, keep); %template palm  points
[Vt_tf_palm,faces_tf_palm] = filter_vertices(Vt_tf, sg_mesh.faces, keep); %transformed template palm points

Vt_palm_idx = Idx(keep);
Normal_t = sg_mesh.normals;
Normal_t_palm = Normal_t(keep, :);
% normal of scan points
for vertexIdx = 1:size(Vsp,1);
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
    
    normals = getNormals(Vsp, F);
    normals_F = normals(row2,:);
    normals_F = sum(normals_F)/norm(normals_F);;
    Normal_sp(vertexIdx,:) = normals_F; % weighted normal vector of vi
end

clear row2 col2 distances ErrorStats i j LI normals nv LMs LMt Template_LM v vertexIdx Vt

%%
O = sg_mesh.vertices;
TF = Vt_tf;
TG = Vsp;
IT = Vt_tf_palm;

figure()
axis equal
axis off
hold on
% original T-points = Gray color
scatter3(O(:,1),O(:,2),O(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
% transformed T-points = Light blue color
scatter3(TF(:,1),TF(:,2),TF(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]); 
% target scan points = Green color
scatter3(TG(:,1),TG(:,2),TG(:,3),'.', 'MarkerEdgeColor',[143/255, 230/255, 143/255]);
% Interest (palm segments) = Red color
scatter3(IT(:,1),IT(:,2),IT(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

clear O TF TG IT

%% Sub-sampling template-palm area
% sub-sample of scan points
n_sub_sample_sp = round(size(Vsp,1)/10);
fprintf('n_sub_sample_sp: %d/%d, demand\n',n_sub_sample_sp,size(Vsp,1));
Idx_sub_sample_sp = randperm(size(Vsp,1),n_sub_sample_sp);
Sub_sample_sp = Vsp(Idx_sub_sample_sp(1:n_sub_sample_sp),:);
% sub-sample of template palm points
n_sub_sample_t_palm = round(size(Vt_palm,1)/10);
fprintf('n_sub_sample_t_palm: %d/%d, supply\n',n_sub_sample_t_palm,size(Vt_palm,1));
Idx_sub_sample_t_palm = randperm(size(Vt_palm,1),n_sub_sample_t_palm);
Sub_sample_t_palm = Vt_palm(Idx_sub_sample_t_palm(1:n_sub_sample_t_palm),:);
% calucation of dummy size for supply (template points)
m = n_sub_sample_sp; % demand, # of original scan = 646
n = n_sub_sample_t_palm; % supply, # of palm = 137
dummysize = m - n; % % 509
fprintf('size of supply dummy: %d\n',dummysize);


%% template palm fitting to raw hand scan

A = zeros(m,m); % m x m matrix 
% A = point to point distnace between template and scan points

for i = 1:n
    for j = 1:m
    cost(i,j) = norm(Sub_sample_t_palm(i,:)-Sub_sample_sp(j,:));
    end
end
cost(n+1:m,:) = 0;
A = cost;
supply = ones(m,1);
demand = ones(1,m);

tic
vogelResult = full(vogels(A,supply,demand))
vogelCost = sum(sum(A.*vogelResult))
optimalSolution = modi(A, vogelResult, supply, demand)
A
fprintf("Optimized cost = %d\n",sum(sum(A.*optimalSolution)))
toc

%% matching - transportation algorithm 
% transportation problem solving algorithmpair  testing

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

%% Others...

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














