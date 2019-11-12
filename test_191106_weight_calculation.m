clc
clear all;

addpath functions;
[V, F, FB, H] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers.mat');

A = V;
figure(1) % point cloud 3D plotting
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
B = centers;
scatter3(B(:,1),B(:,2),B(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

jna = centers(2,:);
jnb = centers(1,:);
vt = [-69.7 -22.5 60.9];
vt_jna = vt - jna
jnb_jna = jnb - jna
delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2

% neighboring vertex info in the FACE column

F2 = F;
LI = F2 == 12;
[row, col] = find(LI);
F2 = F2(row,:);

for i = 1:size(F2,1)*3
    v(i,1) = F2(i);
end
    
nv = 12; % target vertex
for i = 1:size(v,1)
    if v(i) ~= nv
       nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
    end
end

% vertex normal calculation











