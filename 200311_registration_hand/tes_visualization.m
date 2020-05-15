O = vertices_c;
O2 = vertices;
TG = points.vertices;
CX = centers_c;

figure(3)
axis equal
axis off
hold on
%delete(h1);
%delete(h2);
%delete(h3);
% target hand scan = Gray color
h3 = scatter3(TG(:,1),TG(:,2),TG(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);

% Template points = light blue
h1 = scatter3(O(:,1),O(:,2),O(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]);
%h1 = scatter3(O2(:,1),O2(:,2),O2(:,3),'.', 'MarkerEdgeColor',[154/255, 226/255, 247/255]);
h2 = scatter3(CX(:,1),CX(:,2),CX(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);

hold off
