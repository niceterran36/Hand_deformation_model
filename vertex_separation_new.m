clc
clear all;

% Register gptoolbox
addpath(genpath('external'));
addpath 'functions'
[vertices, faces, FaceB, Header] = function_loading_ply_file('hand_meshmodel_190730.ply');

% input points
p1 = [-51.1, -15.29, 24.41];
p2 = [-67.13, -17.17, 17.88];
p3 = [-58.96, -7.68, 20.87];

[a, b, c, d] = generate_plane_3point(p1, p2, p3);

A = vertices;
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a*A(i,1)+b*A(i,2)+c*A(i,3)+d;
    Compare(i) = T;
end 

Compare(Compare>0) = 0; % convert negative value as zero
TT = find(Compare); % find indices of non-zero vertex

Sorted_V = A(TT,:);
Sorted_F = 

    SS = zeros(3283,1);
    for i = 1:3283   
        Dv = a*Sorted_V(i,1) + b*Sorted_V(i,2) + c*Sorted_V(i,3) + d;
        SS(i) = Dv;
    end

a*(-73.13) + b*(-17.77) + c*72.31 + d

face(1,1) 

XX = TT(1) == faces

find(XX(:,1))
find(XX(:,2))
find(XX(:,3))

face(2,2)

for i = 1:3701
S = [TT(i) == faces(:,1)];
Idx(i) = {find(S)'}

    figure(2) % point cloud 3D plotting
    hold on
    axis equal
    scatter3(Sorted_V(:,1),Sorted_V(:,2),Sorted_V(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
    scatter3(Sorted_V(1,1),Sorted_V(1,2),Sorted_V(1,3), '*', 'MarkerEdgeColor',[1, 0, 0])
    hold off























