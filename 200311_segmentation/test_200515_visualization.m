
figure()
axis equal
axis off
hold on
%h3 = scatter3(vi_Dx(:,1),vi_Dx(:,2),vi_Dx(:,3),'.', 'MarkerEdgeColor',[255/255, 180/255, 180/255]);%
h4 = scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
h5 = scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

o_Seg_7
n_Seg_7

figure()
axis equal
axis off
hold on
h3 = scatter3(o_Seg_7(:,1),o_Seg_7(:,2),o_Seg_7(:,3),'.', 'MarkerEdgeColor',[255/255, 180/255, 180/255]);
h4 = scatter3(n_Seg_7(:,1),n_Seg_7(:,2),n_Seg_7(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
%h5 = scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 190/255, 190/255]);
hold off

N2 = V2(N2,:);
N7 = V2(N7,:);


figure()
axis equal
axis off
hold on
h3 = scatter3(N2(:,1),N2(:,2),N2(:,3),'.', 'MarkerEdgeColor',[255/255, 180/255, 180/255]);
h4 = scatter3(N7(:,1),N7(:,2),N7(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
%h5 = scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 190/255, 190/255]);
hold off



keep = ismember(mesh.assignment, 2);
seg2 = mesh.vertices(keep,:);
%% 
load('hy_mesh_n3.mat');
V = mesh.vertices;
v_segment = mesh.assignment;
V2 = V;
% segment = 1;
VLI = [];
for segment=1:22
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
temLI = v_segment == 19; Sg19 = V2(temLI,:);
temLI = v_segment == 20; Sg20 = V2(temLI,:);
temLI = v_segment == 21; Sg21 = V2(temLI,:);
temLI = v_segment == 22; Sg22 = V2(temLI,:);
%%
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
%scatter3(Sg22(:,1),Sg22(:,2),Sg22(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off
%%
figure()
hold on
axis equal
scatter3(Sg7(:,1),Sg7(:,2),Sg7(:,3),'.', 'MarkerEdgeColor',[1, 0, 0])
hold off

%%
figure()
hold on
axis equal
scatter3(Sg22(:,1),Sg22(:,2),Sg22(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

%%
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

%% 3D Rendering

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


