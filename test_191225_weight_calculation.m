clc; clear all;
addpath functions;
addpath(genpath('external'));
load('sg_mesh.mat');
centers = sg_mesh.centers; v_segment = sg_mesh.assignment; V = sg_mesh.vertices; F = sg_mesh.faces;
S = sg_mesh.bonestructure;

Sg1 = sg_mesh.segment.Sg1;Sg2 = sg_mesh.segment.Sg2;Sg3 = sg_mesh.segment.Sg3;Sg4 = sg_mesh.segment.Sg4;Sg5 = sg_mesh.segment.Sg5;Sg6 = sg_mesh.segment.Sg6;Sg7 = sg_mesh.segment.Sg7;Sg8 = sg_mesh.segment.Sg8;
Sg9 = sg_mesh.segment.Sg9;Sg10 = sg_mesh.segment.Sg10;Sg11 = sg_mesh.segment.Sg11;Sg12 = sg_mesh.segment.Sg12;Sg13 = sg_mesh.segment.Sg13;Sg14 = sg_mesh.segment.Sg14;Sg15 = sg_mesh.segment.Sg15;
Sg16 = sg_mesh.segment.Sg16; Sg17 = sg_mesh.segment.Sg17; Sg18 = sg_mesh.segment.Sg18; Sg19 = sg_mesh.segment.Sg19; Sg20 = sg_mesh.segment.Sg20; Sg21 = sg_mesh.segment.Sg21;

%% Visualization mesh
A = centers;
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
scatter3(Sg19(:,1),Sg19(:,2),Sg19(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg20(:,1),Sg20(:,2),Sg20(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg21(:,1),Sg21(:,2),Sg21(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
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
%scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
hold off

%% vertex id finder % drawer

pick = [4.0035 -14.7856 42.0156]

t = zeros(size(V,1),2); 
for i = 1:size(V,1)
    t(i,1) = i;
    t(i,2) = norm(V(i,1:3)-pick);
end 
mindist_pt = min(t(:,2));
[vertexIdx ,col] = find(t(:,2) == mindist_pt);
main_segment = v_segment(vertexIdx);
fprintf('VertexIdx = %.0f, Current segment = %.0f\n',vertexIdx, main_segment)

if main_segment == 1
    SgV = Sg1;
    jna = centers(22,:); 
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
% function, initial parameter setting
a = 1.3;
c = 0.25;
fx = a*exp(-((x-0.5)^2)/(2*c^2));

W = zeros(size(V,1),21);

vertexIdx = 1;

% main influence joint = jna of assigned segment