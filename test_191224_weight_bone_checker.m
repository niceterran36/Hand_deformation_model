clc
clear all;
addpath(genpath('external'));
addpath 'functions';

%% load data
mesh = load('tr_mesh.mat');
V = mesh.transformed.vertices;
F = mesh.transformed.faces;
axes = bone_axes(mesh.transformed.spheres);

% CoR Information
centers = zeros(30,3);
for i = 1:size(axes,2)
centers(i,:) = (axes{1,i}(1:3,4))';
end

% Wrist = centers(2,:)
% Digit1 CMC, IP, DIP = centers(3,4,5,:)
% Digit2 MCP, PIP, DIP = centers(6,7,8,:)
% Digit3 MCP, PIP, DIP = centers(9,10,11,:)
% Digit4 MCP, PIP, DIP = centers(12,13,14,:)
% Digit5 MCP, PIP, DIP = centers(16,17,18,:)

A = centers;
figure() % point cloud 3D plotting
hold on
axis equal
plot3(A(:,1),A(:,2),A(:,3),'b*') % CoR plotting
% Link between CoRs
plot3(A(1:2,1), A(1:2,2), A(1:2,3),'k-')
plot3(A(2:5,1), A(2:5,2), A(2:5,3),'b-')
plot3(A(6:8,1), A(6:8,2), A(6:8,3),'k-')
plot3(A(9:11,1), A(9:11,2), A(9:11,3),'k-')
plot3(A(12:14,1), A(12:14,2), A(12:14,3),'k-')
plot3(A(16:18,1), A(16:18,2), A(16:18,3),'k-')
plot3(A([2 6],1),A([2 6],2),A([2 6],3),'b-')
plot3(A([2 9],1),A([2 9],2),A([2 9],3),'b-')
plot3(A([2 12],1),A([2 12],2),A([2 12],3),'b-')
plot3(A([2 16],1),A([2 16],2),A([2 16],3),'b-')
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
hold off

%% Segment visualization
V2 = V;
VLI = [];
v_segment = mesh.transformed.assignments;
for segment=1:18
temLI = v_segment == segment;
VLI = [VLI temLI];
end 

temLI = v_segment == 1; Sg1 = V2(temLI,:);
temLI = v_segment == 2; Sg2 = V2(temLI,:);
temLI = v_segment == 3; Sg3 = V2(temLI,:);
temLI = v_segment == 4; Sg4 = V2(temLI,:);
temLI = v_segment == 5; Sg5 = V2(temLI,:);
temLI = v_segment == 6; Sg6 = V2(temLI,:);
temLI = v_segment == 7; Sg7 = V2(temLI,:);
temLI = v_segment == 8; Sg8 = V2(temLI,:);
temLI = v_segment == 9; Sg9 = V2(temLI,:);
temLI = v_segment == 10; Sg10 = V2(temLI,:);
temLI = v_segment == 11; Sg11 = V2(temLI,:);
temLI = v_segment == 12; Sg12 = V2(temLI,:);
temLI = v_segment == 13; Sg13 = V2(temLI,:);
temLI = v_segment == 14; Sg14 = V2(temLI,:);
temLI = v_segment == 15; Sg15 = V2(temLI,:);
temLI = v_segment == 16; Sg16 = V2(temLI,:);
temLI = v_segment == 17; Sg17 = V2(temLI,:);
temLI = v_segment == 18; Sg18 = V2(temLI,:);

% segment color re-allocation

figure()
hold on
axis equal
scatter3(Sg1(:,1),Sg1(:,2),Sg1(:,3),'.', 'MarkerEdgeColor',[16/255, 241/255, 255/255])
scatter3(Sg2(:,1),Sg2(:,2),Sg2(:,3),'.', 'MarkerEdgeColor',[213/255, 42/255, 219/255])
scatter3(Sg3(:,1),Sg3(:,2),Sg3(:,3),'.', 'MarkerEdgeColor',[233/255, 30/255, 68/255])
scatter3(Sg4(:,1),Sg4(:,2),Sg4(:,3),'.', 'MarkerEdgeColor',[179/255, 59/255, 235/255])
scatter3(Sg5(:,1),Sg5(:,2),Sg5(:,3),'.', 'MarkerEdgeColor',[69/255, 204/255, 104/255])
scatter3(Sg6(:,1),Sg6(:,2),Sg6(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
scatter3(Sg7(:,1),Sg7(:,2),Sg7(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg8(:,1),Sg8(:,2),Sg8(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg9(:,1),Sg9(:,2),Sg9(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
scatter3(Sg10(:,1),Sg10(:,2),Sg10(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg11(:,1),Sg11(:,2),Sg11(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg12(:,1),Sg12(:,2),Sg12(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
scatter3(Sg13(:,1),Sg13(:,2),Sg13(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg14(:,1),Sg14(:,2),Sg14(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg15(:,1),Sg15(:,2),Sg15(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
scatter3(Sg16(:,1),Sg16(:,2),Sg16(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg17(:,1),Sg17(:,2),Sg17(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg18(:,1),Sg18(:,2),Sg18(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
hold off

%% vertex id finder % drawer

% pick = [-5.3197 2.7587 2.5665] % w = 1
pick = [-4.4061 0.6770 2.0835]

t = zeros(size(V,1),2); 
for i = 1:size(V,1)
    t(i,1) = i;
    t(i,2) = norm(V(i,1:3)-pick);
end 
mindist_pt = min(t(:,2));
[vertexIdx ,col] = find(t(:,2) == mindist_pt);
main_segment = mesh.transformed.assignments(vertexIdx);
fprintf('VertexIdx = %.0f, Current segment = %.0f\n',vertexIdx, main_segment)

% row and column information of target, wc_t = weight column for target
wc_t = zeros(18,2); wc_t(:,1) = [1:18]';
wc_t(:,2) = mesh.transformed.weights(vertexIdx,:)';
LIX = wc_t(:,2) > 0;
wc_t = wc_t(LIX,:);

% weight print per joint
if size(wc_t,1) == 2
   fprintf('Joint%.0f = %3.2f, Joint%.0f = %3.2f\n',wc_t(1,1),wc_t(1,2),wc_t(2,1),wc_t(2,2))
elseif size(wc_t,1) == 3
   fprintf('Joint%.0f = %3.2f, Joint%.0f = %3.2f,Joint%.0f = %3.2f\n',wc_t(1,1),wc_t(1,2),wc_t(2,1),wc_t(2,2),wc_t(3,1),wc_t(3,2))
elseif size(wc_t,1) == 4
   fprintf('Joint%.0f = %3.2f, Joint%.0f = %3.2f,Joint%.0f = %3.2f, Joint%.0f = %3.2f\n',wc_t(1,1),wc_t(1,2),wc_t(2,1),wc_t(2,2),wc_t(3,1),wc_t(3,2),wc_t(4,1),wc_t(4,2))
else
   fprintf('Joint%.0f = %3.2f\n',wc_t(1,1),wc_t(1,2))
end 

if main_segment == 1
    SgV = Sg1;
elseif main_segment == 2
    SgV = Sg2;
elseif main_segment == 3
    SgV = Sg3;
elseif main_segment == 4
    SgV = Sg4;
elseif main_segment == 5
    SgV = Sg5;
elseif main_segment == 6
    SgV = Sg6;
elseif main_segment == 7
    SgV = Sg7;
elseif main_segment == 8
    SgV = Sg8;
elseif main_segment == 9
    SgV = Sg9;
elseif main_segment == 10
    SgV = Sg10;
elseif main_segment == 11
    SgV = Sg11;
elseif main_segment == 12
    SgV = Sg12;
elseif main_segment == 13
    SgV = Sg13;
elseif main_segment == 14
    SgV = Sg14;
elseif main_segment == 15
    SgV = Sg15;   
elseif main_segment == 16
    SgV = Sg16;
elseif main_segment == 17
    SgV = Sg17;
elseif main_segment == 18
    SgV = Sg18;
else
    SgV = [];
end

figure() % point cloud 3D plotting
hold on
axis equal
plot3(A(:,1),A(:,2),A(:,3),'b*') % CoR plotting
% Link between CoRs
plot3(A(1:2,1), A(1:2,2), A(1:2,3),'k-')
plot3(A(2:5,1), A(2:5,2), A(2:5,3),'b-')
plot3(A(6:8,1), A(6:8,2), A(6:8,3),'k-')
plot3(A(9:11,1), A(9:11,2), A(9:11,3),'k-')
plot3(A(12:14,1), A(12:14,2), A(12:14,3),'k-')
plot3(A(16:18,1), A(16:18,2), A(16:18,3),'k-')
plot3(A([2 6],1),A([2 6],2),A([2 6],3),'b-')
plot3(A([2 9],1),A([2 9],2),A([2 9],3),'b-')
plot3(A([2 12],1),A([2 12],2),A([2 12],3),'b-')
plot3(A([2 16],1),A([2 16],2),A([2 16],3),'b-')
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
scatter3(V(vertexIdx,1),V(vertexIdx,2),V(vertexIdx,3), 'o', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
scatter3(A(wc_t(:,1),1),A(wc_t(:,1),2),A(wc_t(:,1),3), 'o', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
scatter3(SgV(:,1),SgV(:,2),SgV(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
hold off




%%

for i = 1:3045
    if FaceColor_mx(i,1) == 14
       FaceColor_mx(i,2:4) = [242 62 27];
    else
       FaceColor_mx(i,2:4) = [0 0 0];
    end 
end 

FaceColor_mx_n = FaceColor_mx(:,2:4);


h1 = trisurf(F, V(:,1), V(:,2), V(:,3));
    set(h1, 'FaceColor',[1 0.88 0.77]) %face color matrix 생성하여 개별값 삽입 가능
    set(h1, 'EdgeColor', 'none');
    set(h1, 'facealpha', 1);
    
    view(2);
    axis equal;
    light('Position', [3 5 7], 'Style', 'infinite');
    lighting gouraud;
    material dull;
    
    hold on;

















