
A1 = HY_pos1_vertices;
A2 = HY_pos2_vertices;
A3 = HY_pos3_vertices;
A4 = HY_pos4_vertices;
A5 = HY_pos5_vertices;
A6 = HY_pos6_vertices;
A7 = HY_pos7_vertices;
A8 = HY_pos8_vertices;


figure()
hold on;
axis equal
axis off
scatter3(A1(:,1),A1(:,2),A1(:,3),'.', 'MarkerEdgeColor',[213/255, 213/255, 213/255]);
scatter3(A1(LMt_Idx,1),A1(LMt_Idx,2),A1(LMt_Idx,3),'*r');
% scatter3(A2(:,1),A2(:,2),A2(:,3),'.', 'MarkerEdgeColor',[213/255, 213/255, 213/255]);
% scatter3(A2(LMt_Idx,1),A2(LMt_Idx,2),A2(LMt_Idx,3),'*r');
% scatter3(A3(:,1),A3(:,2),A3(:,3),'.', 'MarkerEdgeColor',[213/255, 213/255, 213/255]);
% scatter3(A3(LMt_Idx,1),A3(LMt_Idx,2),A3(LMt_Idx,3),'*r');
% scatter3(A4(:,1),A4(:,2),A4(:,3),'.', 'MarkerEdgeColor',[213/255, 213/255, 213/255]);
% scatter3(A4(LMt_Idx,1),A4(LMt_Idx,2),A4(LMt_Idx,3),'*r');
% scatter3(A5(:,1),A5(:,2),A5(:,3),'.', 'MarkerEdgeColor',[213/255, 213/255, 213/255]);
% scatter3(A5(LMt_Idx,1),A5(LMt_Idx,2),A5(LMt_Idx,3),'*r');

hold off