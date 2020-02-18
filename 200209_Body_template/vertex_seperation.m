% Register gptoolbox
addpath(genpath('external'));
addpath 'functions'


[vertices, faces, FaceB, Header] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers.mat')

A = vertices;
B = centers;


%[a, b, c, d] = generate_plane_3point(p1, p2, p3) % ax+by+cz+d = 0

% generate plane from D5 Dip joint near points
% load('pointondip.mat')
ptondip = [-63.53,-14.78,52.63; -74,-22.73,51.55; -62.88,-17.66,53.55];
p1 = ptondip(1,:); p2 = ptondip(2,:);, p3 = ptondip(3,:);
[a, b, c, d] = generate_plane_3point(p1, p2, p3); % plane is defined as ax+by+cz+d = 0

% D5 joint CoR based vector calculation
D5 = centers(1:4,:);
v1 = [D5(1)-D5(2) D5(5)-D5(6) D5(9)-D5(10)];
v2 = [D5(2)-D5(3) D5(6)-D5(7) D5(10)-D5(11)];
v3 = [D5(3)-D5(4) D5(7)-D5(8) D5(11)-D5(12)];
v = v1+v2+v3;

% seperate vertex using 3D plane
A = vertices;
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a*A(i,1)+b*A(i,2)+c*A(i,3)+d;
    Compare(i) = T;
end 

Compare(Compare<0) = 0; % convert negative value as zero
TT = find(Compare); % find indices of non-zero vertex

D5_medium = (D5(1,:) + D5(2,:))/2;
% threshold1 = sqrt((edge(1)-D5_medium(1))^2 + (edge(2)-D5_medium(2))^2 + (edge(3)-D5_medium(3))^2);
% threshold2 = sqrt((notd5(1)-D5_medium(1))^2 + (notd5(2)-D5_medium(2))^2 + (notd5(3)-D5_medium(3))^2);
threshold = 15;

for i = 1:size(TT,1)
TT(i,2) = sqrt((D5_medium(1)-A(TT(i),1))^2 + (D5_medium(2)-A(TT(i),2))^2 + (D5_medium(3)-A(TT(i),3))^2);
end

for i = 1:size(TT,1)
    if TT(i,2) > threshold
       TT(i,:) = 0;
    end
end

w = TT(TT(:,2) > 0);

for i = 1:size(w,1)
D5_points(i,:) = A(w(i),:);
end

% non-D5 vertex sorting
A1 = A(:,1); A2 = A(:,2); A3 = A(:,3); 
x = ones(size(A,1),1);
for i = 1:size(w,1);
    x(w(i)) = 0;
end 
AA = logical(x);
ATT = [A1(AA) A2(AA) A3(AA)];


%% visualization
figure(1) % point cloud 3D plotting
    hold on
    axis equal
    A = vertices;
    scatter3(D5_points(:,1),D5_points(:,2),D5_points(:,3),'.', 'MarkerEdgeColor',[255/255, 0, 0])
    B = centers;
    scatter3(B(:,1),B(:,2),B(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
    scatter3(D5_medium(1),D5_medium(2),D5_medium(3),'.', 'MarkerEdgeColor',[100/255, 240/255, 122/255])
    scatter3(ATT(:,1),ATT(:,2),ATT(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
    hold off
    
    figure(2) % point cloud 3D plotting
    hold on
    axis equal
    A = vertices;
    scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
    B = centers;
    scatter3(B(:,1),B(:,2),B(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
    scatter3(D5_medium(1),D5_medium(2),D5_medium(3),'.', 'MarkerEdgeColor',[100/255, 240/255, 122/255])
    hold off
