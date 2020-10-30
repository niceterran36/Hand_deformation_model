% vertices segmentation for dorsal, palmar

addpath(genpath('external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('LM_for_dorsal_palmar')
LMs = function_get_LM_from_iges('D2seg1.igs');

% assignment information
% D2 distal phalanx: assignment 11
% D2 middle phalanx: assignment 10
% D2 proximal phalanx: asisgnment 9

figure()
axis equal
axis off
hold on
view(-7,7);
scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(LMs(:,1),LMs(:,2),LMs(:,3),'*','MarkerEdgeColor',[242/255, 150/255, 97/255]);
scatter3(Sg11(:,1),Sg11(:,2),Sg11(:,3),'.','MarkerEdgeColor',[243/255, 97/255, 166/255]);
hold off

% input points
p1 = LMs(1,:);
p2 = LMs(2,:);
p3 = LMs(3,:);
[a, b, c, d] = generate_plane_3point(p1, p2, p3);
PL_com = zeros(1,4);
PL_com = [a, b, c, d];
clear a b c d

LIX = mesh.assignment == 11;
V_IDX = [1:size(mesh.vertices,1)]';
Sg11 = vertices(LIX,:);
Sg11_idx = V_IDX(LIX,:);

% testing value
stone = [8.6169 -7.8574, 95.0876];
T = PL_com(1)*stone(1)+PL_com(2)*stone(2)+PL_com(3)*stone(3)+PL_com(4);

% =================================== start here

A = vertices;
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a*A(i,1)+b*A(i,2)+c*A(i,3)+d;
    Compare(i) = T;
end 

Compare(Compare>0) = 0; % convert negative value as zero
TT = find(Compare); % find indices of non-zero vertex

Sorted_V = A(TT,:);
 
% Distance calculation - 1 vertex
% P0 = p3; % point on the plane 
% Q = [-74.72, -16.89, 71.51]; % any point Q above the plane
% v_n = [a, b, c]; % othogornal vector of plane 
% v_b = Q - P0;  % vector between P0 and Q
% Dstc = sqrt((dot(v_n, v_b))^2)/sqrt(a^2+b^2+c^2) % distance function between Plane and Q

% Distance calculation between selected vertex(vi) and Plane 
P0 = p3;
v_n = [a, b, c]; 
TT2 = TT;
for i = 1:size(TT,1)
    Q = [A(TT(i),1), A(TT(i),2), A(TT(i),3)];
    v_b = Q - P0; 
    Dstc = sqrt((dot(v_n, v_b))^2)/sqrt(a^2+b^2+c^2);
    TT2(i,2) = Dstc;
end

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























