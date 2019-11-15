clc
clear all;

addpath functions;
addpath(genpath('external'));
[V, F, FB, H] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers.mat');
% 1~4: little, 5~8: ring, 9~12: middle, 13~16: index, 17~20: thumb, 
% 22: wrist center
A = centers;
C = V;
figure(2) % point cloud 3D plotting
hold on
axis equal
plot3(A(:,1),A(:,2),A(:,3),'b*')
plot3(A(1:4,1), A(1:4,2), A(1:4,3),'k-')
plot3(A(5:8,1), A(5:8,2), A(5:8,3),'k-')
plot3(A(9:12,1), A(9:12,2), A(9:12,3),'k-')
plot3(A(13:16,1), A(13:16,2), A(13:16,3),'k-')
plot3(A(17:20,1), A(17:20,2), A(17:20,3),'k-')
plot3(A([4 22],1),A([4 22],2),A([4 22],3),'b-')
plot3(A([8 22],1),A([8 22],2),A([8 22],3),'b-')
plot3(A([12 22],1),A([12 22],2),A([12 22],3),'b-')
plot3(A([16 22],1),A([16 22],2),A([16 22],3),'b-')
plot3(A([20 22],1),A([20 22],2),A([20 22],3),'b-')
plot3(A([22 27],1),A([22 27],2),A([22 27],3),'k-')
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
scatter3(C(:,1),C(:,2),C(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
scatter3(A([1:20, 22],1),A([1:20, 22],2),A([1:20, 22],3), 'o', 'MarkerEdgeColor',[255/255, 0/255, 0/255])


hold off



C = V;
figure(1) % point cloud 3D plotting
hold on
axis equal
scatter3(C(:,1),C(:,2),C(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
B = centers;
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

jna = centers(3,:);
jnb = centers(2,:);
%vt = [-69.7 -22.5 60.9]; % after the segment 
vt = [-61.20 -20.35 44.06]; % between the segment
%vt = [-47.85 -0.99 -10.44] % before the segment
vt_jna = vt - jna
jnb_jna = jnb - jna
delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2

% distance of projection vector from vi to segment 
% vt: target vertex 
av = vt-jna; bv = jnb-jna; cv = dot(av,bv)/norm(bv); % vector define
dv1 = sqrt(norm(av)^2 - norm(cv)^2); % distance by trigonometric functions
vd = av - dot(av,bv)/norm(bv)^2 * bv; % distance by vector & norm 
dv2 = norm(vd);

% neighboring vertex info in the FACE column
vertexIdx = 12;

F2 = F;
LI = F2 == vertexIdx;
[row, col] = find(LI);
F2 = F2(row,:);

for i = 1:size(F2,1)*3
    v(i,1) = F2(i);
end
    
nv = vertexIdx; % target vertex
for i = 1:size(v,1)
    if v(i) ~= nv
       nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
    end
end

% vertex normal calculation (unit vector by sum of the around face normal)

normals = getNormals(V, F);
normals_F = normals(row,:);
normals_F = sum(normals_F);
normals_F = normals_F/norm(normals_F);






